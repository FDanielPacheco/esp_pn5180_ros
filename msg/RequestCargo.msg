# This message is used to request RFID operations with Cargo tag mode.
uint8 request_id        # The request identification
uint8 operation         # Type of operation, options can be: read - 0 or write - 1
TagCargo arg            # In case the robot selects 1, then can provide content to write on the tag with these arguments.