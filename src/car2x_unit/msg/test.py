import os
import re

# Specify the directory you want to check
directory = '/home/huiyu/group1_service/src/car2x_unit/msg'

# Specify the path to your .proto file
proto_file_path = '/home/huiyu/etsi_proto_msg/etsi_its_cdd.proto'

# Read the .proto file and extract all message names
with open(proto_file_path, 'r') as f:
    proto_content = f.read()
    # Regular expression to match message definitions
    regex = re.compile(r'\b(message|enum)\s+(\w+)\b', re.IGNORECASE)
    messages = regex.findall(proto_content)

# Loop over the list of messages
for message in messages:
    # Construct the expected .msg file name
    msg_file_name = message[1] + '.msg'
    # print(msg_file_name)
    # Construct the expected .msg file path
    msg_file_path = os.path.join(directory, msg_file_name)
    # Check if the .msg file exists
    if not os.path.isfile(msg_file_path):
        print(f"File '{msg_file_name}' not found.")
