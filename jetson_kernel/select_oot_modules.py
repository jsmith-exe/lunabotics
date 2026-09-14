#!/usr/bin/env python3
"""Choose which NVIDIA out-of-tree kernel modules to install for the IFB kernel.

Enabling IFB also enables CONFIG_NET_REDIRECT, which changes struct sk_buff, so the symbol CRCs of anything
that touches networking change. A stock module that imports one of those symbols won't load on the new
kernel and must be replaced with one rebuilt from source. Every other module is reused as-is from the stock
install, so it stays exactly what NVIDIA shipped for this JetPack release.

Prints one tab-separated line per module: <path under updates/> <file to install> <stock|rebuilt>
"""
import argparse
import collections
import glob
import os
import subprocess
import sys


def read_symvers(path):
    exports = {}
    with open(path) as f:
        for line in f:
            crc, symbol = line.split("\t")[:2]
            exports[symbol] = int(crc, 16)
    return exports


def module_exports(ko):
    # Exported symbols' CRCs are stored in the module as __crc_<symbol> symbols
    output = subprocess.run(["nm", ko], capture_output=True, text=True, check=True).stdout
    exports = {}
    for line in output.splitlines():
        parts = line.split()
        if len(parts) == 3 and parts[2].startswith("__crc_"):
            exports[parts[2][len("__crc_"):]] = int(parts[0], 16)
    return exports


def module_imports(ko):
    output = subprocess.run(["modprobe", "--dump-modversions", ko],
                            capture_output=True, text=True, check=True).stdout
    return {symbol: int(crc, 16) for crc, symbol in (line.split() for line in output.splitlines())}


def srcversion(ko):
    return subprocess.run(["modinfo", "-F", "srcversion", ko], capture_output=True, text=True).stdout.strip()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--kernel-symvers", required=True, help="Module.symvers of the new kernel build")
    parser.add_argument("--stock-updates", required=True, help="e.g. /lib/modules/5.15.148-tegra/updates")
    parser.add_argument("--rebuilt-dir", action="append", required=True,
                        help="directory to search for rebuilt .ko files (repeatable)")
    args = parser.parse_args()

    kernel = read_symvers(args.kernel_symvers)
    stock = {os.path.relpath(p, args.stock_updates): p
             for p in glob.glob(f"{args.stock_updates}/**/*.ko", recursive=True)}

    rebuilt_by_name = collections.defaultdict(list)
    for directory in args.rebuilt_dir:
        for p in glob.glob(f"{directory}/**/*.ko", recursive=True):
            rebuilt_by_name[os.path.basename(p)].append(p)
    rebuilt = {}
    for rel in stock:
        matches = rebuilt_by_name.get(os.path.basename(rel), [])
        if len(matches) == 1:
            rebuilt[rel] = matches[0]
        elif len(matches) > 1:
            print(f"warning: several rebuilt files match {rel}, not using any: {matches}", file=sys.stderr)

    choice = {rel: "stock" for rel in stock}

    def path(rel):
        return stock[rel] if choice[rel] == "stock" else rebuilt[rel]

    exports_cache, imports_cache = {}, {}

    def exports(ko):
        return exports_cache.setdefault(ko, module_exports(ko))

    def imports(ko):
        return imports_cache.setdefault(ko, module_imports(ko))

    # Switch modules to rebuilt copies until every import's CRC matches the symbol that will provide it
    while True:
        providers = {symbol: (crc, None) for symbol, crc in kernel.items()}
        for rel in stock:
            for symbol, crc in exports(path(rel)).items():
                providers[symbol] = (crc, rel)

        changed = False
        problems = []
        for rel in stock:
            for symbol, crc in imports(path(rel)).items():
                if symbol not in providers or providers[symbol][0] == crc:
                    continue  # Unresolved symbols are reported by depmod instead
                provider = providers[symbol][1]
                if choice[rel] == "stock" and rel in rebuilt:
                    choice[rel] = "rebuilt"
                    changed = True
                    break
                if provider is not None and choice[provider] == "stock" and provider in rebuilt:
                    choice[provider] = "rebuilt"
                    changed = True
                    break
                problems.append(f"{rel}: CRC mismatch for {symbol} and no rebuilt module to fix it")
                break
        if not changed:
            break

    if problems:
        print("\n".join(problems), file=sys.stderr)
        sys.exit(1)

    counts = collections.Counter(choice.values())
    print(f"{counts['stock']} stock modules reused, {counts['rebuilt']} rebuilt", file=sys.stderr)
    # srcversion is a checksum of a module's source files, but only modules with a MODULE_VERSION have one
    with_srcversion = [rel for rel in rebuilt if choice[rel] == "rebuilt" and srcversion(stock[rel])]
    same_source = [rel for rel in with_srcversion if srcversion(stock[rel]) == srcversion(rebuilt[rel])]
    print(f"{len(same_source)}/{len(with_srcversion)} rebuilt modules with a srcversion match stock", file=sys.stderr)
    for rel in sorted(set(with_srcversion) - set(same_source)):
        print(f"  srcversion differs: {rel}", file=sys.stderr)
    for rel in sorted(stock):
        print(f"{rel}\t{path(rel)}\t{choice[rel]}")


if __name__ == "__main__":
    main()
