"""
Helper functions for generating CAN messages (FRC IDs)
"""

# FRC CAN ID layout
#
# SPARK MAX uses 29-bit extended CAN IDs following the FRC CAN specification.
# The 29 bits are divided into fields:
#   bits 28-24  device type  (5 bits)  — what kind of device this is
#   bits 23-16  manufacturer (8 bits)  — who made it
#   bits 15-10  api class    (6 bits)  — command/status category
#   bits  9- 6  api index    (4 bits)  — specific command within the class
#   bits  5- 0  device id    (6 bits)  — which physical controller (1-63)
#
# For all SPARK MAX frames:
#   device type  = 0x02  (motor controller)
#   manufacturer = 0x05  (REV Robotics)


def create_frc_id(api_class, api_index, device_id):
    """
    Build a 29-bit FRC extended CAN ID for a REV SPARK MAX.

    Hardcodes device_type=0x02 (motor controller) and manufacturer=0x05 (REV).
    The api_class and api_index together identify the type of message;
    device_id identifies which physical SPARK MAX on the bus (1-63).
    """
    return (
            (0x02 << 24) |                     # device type: motor controller
            (0x05 << 16) |                     # manufacturer: REV Robotics
            ((api_class  & 0x3F) << 10) |      # 6-bit api class
            ((api_index  & 0x0F) <<  6) |      # 4-bit api index
            ((device_id  & 0x3F) <<  0)        # 6-bit device id
    )

def create_packed_id(api_id, dev_id):
    """
    Build a CAN ID from a packed 8-bit api_id as used in CAN_comms.cpp.

    In the C++ code, SPARKMAX_API_DUTY_CYCLE_SET=0x02 and
    SPARKMAX_API_VELOCITY_SET=0x12 are split by treating the upper nibble
    as api_class and the lower nibble as api_index:
      0x02 -> class=0, index=2  (duty cycle set)
      0x12 -> class=1, index=2  (velocity set)
    """
    return create_frc_id((api_id >> 4) & 0x3F, api_id & 0x0F, dev_id)

# Non-RIO heartbeat: api_class=0x16, api_index=0x00, device_id=0x00 (broadcast)
# The plugin sends this periodically to tell SPARK MAXs a non-FRC controller
# is present. Without it, SPARK MAXs disable their outputs after ~100ms.
HEARTBEAT_ID = create_frc_id(0x0B, 0x02, 0x00)
