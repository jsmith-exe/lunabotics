from pathlib import Path
from typing import Iterable
from os import environ, listdir

import urwid
from urwid import AttrMap as wrap_with_attr

palette = [
    ("title", "dark red", "black"),
    ("current_option", "light gray", "black"),
    ("btn", "white", "black"),
    ("reversed_btn", "dark red", "dark gray"),
]

current_dds = environ.get("CURRENT_DDS", "Unset")
qpl_project = environ.get("QPL_PROJECT", None)
if qpl_project is None:
    raise ValueError("QPL_PROJECT environment variable is not set. Please set it to the path of your QPL project.")
dds_dir = Path(qpl_project) / "dds"
dds_config_file = dds_dir / ".current_dds"

def get_current_dds() -> str:
    return current_dds

def get_dds_options() -> list[str]:
    all_filenames = listdir(dds_dir)
    dds_filenames = filter(lambda filename: filename.endswith(".xml"), all_filenames)
    return list(dds_filenames) + ["Unset"]

def set_dds(target_dds_name: str) -> None:
    """ Sets the current environment variables and the .current_dds file to the target DDS. Pass blank string to unset DDS. """
    dds_path = dds_dir / target_dds_name
    env_variables = {
        "CURRENT_DDS": target_dds_name,
        "RMW_IMPLEMENTATION": get_dds_type(target_dds_name),
        "CYCLONEDDS_URI": f"file://{dds_path}"
    }
    env_file_lines = [f"{key}={value}\n" for key, value in env_variables.items()]
    with open(dds_config_file, "w") as dds_config:
        dds_config.writelines(env_file_lines)

def get_dds_type(dds_file_name: str) -> str:
    if dds_file_name.startswith("cyclone"):
        return "rmw_cyclonedds_cpp"
    return ""


# UI (forget Single Responsibility Principle)
def exit_on_q(key: str) -> None:
    if key in {"q", "Q"}:
        raise urwid.ExitMainLoop()

def menu(title: str, current_choice: str, choices: Iterable[str]) -> urwid.ListBox:
    """ Creates a list of buttons. """
    # Create title and add title tag to it
    title = wrap_with_attr(urwid.Text(title), "title")
    current_option = wrap_with_attr(urwid.Text(f"Current option: {current_choice}"), "current_option")
    body = [title, current_option, urwid.Divider()]
    for choice in choices:
        button = urwid.Button(choice)
        urwid.connect_signal(button, "click", item_chosen, choice)
        btn_wrapped = wrap_with_attr(button, "btn", focus_map="reversed_btn")
        body.append(btn_wrapped)
    return urwid.ListBox(urwid.SimpleFocusListWalker(body))

def item_chosen(button: urwid.Button, choice: str) -> None:
    if choice == "Unset":
        set_dds("")
    else:
        set_dds(choice)
    raise urwid.ExitMainLoop()

main = urwid.Padding(menu("DDS Selector", get_current_dds(), get_dds_options()), left=2, right=2)
urwid.MainLoop(main, palette=palette, unhandled_input=exit_on_q).run()
