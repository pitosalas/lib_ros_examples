# Initialize an empty dictionary to store variables and their values
variables = {}

def get_value(token):
    # Check if the token is a variable name
    if token in variables:
        return float(variables[token])
    # If not, assume it's a float value
    try:
        return float(token)
    except ValueError:
        return None

# Define the command table
command_table = {
    "reset": {
        "args": [],
        "description": "Reset the state",
        "handler": lambda args: variables.clear(),
    },
    "set": {
        "args": ["<variable>", "<value>"],
        "description": "Set a variable to a value",
        "handler": lambda args: variables.update({args[0]: get_value(args[1])}) if len(args) == 2 else None,
    },
    "show": {
        "args": ["[<variable>]"],
        "description": "Show all variables or the value of a specific variable",
        "handler": lambda args: print_variables() if len(args) == 0 else print_variable(args[0]),
    },
    "move": {
        "args": ["<forward_speed>", "<distance>"],
        "description": "Move with a given forward speed and distance",
        "handler": lambda args: move(get_value(args[0]), get_value(args[1])) if len(args) == 2 else None,
    },
    "help": {
        "args": [],
        "description": "Show available commands",
        "handler": lambda args: print_commands(),
    },
    "quit": {
        "args": [],
        "description": "Quit the program",
        "handler": lambda args: exit(),
    },
}

def print_commands():
    print("Available commands:")
    for command, info in command_table.items():
        args = " ".join(info["args"])
        print(f"- {command} {args}: {info['description']}")

def print_variables():
    if variables:
        print("Variables:")
        for name, value in variables.items():
            print(f"{name}: {value}")
    else:
        print("No variables have been set.")

def print_variable(variable_name):
    if variable_name in variables:
        print(f"The value of '{variable_name}' is '{variables[variable_name]}'.")
    else:
        print(f"Variable '{variable_name}' has not been set.")

def move(forward_speed, distance):
    if forward_speed is not None and distance is not None:
        print(f"Moving with forward speed '{forward_speed}' for distance '{distance}'.")
        # Implement the logic for moving here
    else:
        print("Invalid value. Please provide valid floats or existing variables.")

while True:
    # Prompt the user for a command
    command = input(">> ")

    # Split the command into tokens
    tokens = command.split()

    # Check if the command exists in the command table
    if tokens[0] in command_table:
        command_info = command_table[tokens[0]]
        expected_args = command_info["args"]

        if len(tokens[1:]) != len(expected_args):
            print(f"Invalid command. Usage: {tokens[0]} {' '.join(expected_args)}")
        else:
            command_info["handler"](tokens[1:])
    else:
        print("Invalid command. Please try again.")

