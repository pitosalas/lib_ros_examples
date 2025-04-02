#!/usr/bin/env python3
"""
Package of very simple parsing used in Robogym and friends
"""
from json import JSONDecodeError, loads
from prompt_toolkit import PromptSession


class Parser:
    """General purpose simple interactive command processor"""

    def __init__(self, symbol_table, command_table):
        # Initialize an empty dictionary to store variables and their values
        self.session = None
        self.command_info = None
        self.symbol_table = symbol_table
        self.session = PromptSession()
        self.command_input = None

        # Define the built-in command table
        self.local_command_table = {
            "help": {
                "args": [],
                "nargs": 0,
                "description": "Show available commands",
                "handler": self.print_commands,
            },
            "set": {
                "args": ["<variable>", "<value>"],
                "nargs": 2,
                "description": "Set a variable to a value",
                "handler": self.set_variable,
                "usage": "Invalid command. Usage: set <varname> <value>",
            },
            "show": {
                "nargs": 0,
                "args": ["[<variable>]"],
                "description": "Show all variables or the value of a specific variable",
                "handler": self.show_variable,
                "usage": "Invalid command. Usage: show <varname>",

            },
            "reset": {
                "args": [],
                "nargs": 0,
                "description": "Reset the state",
                "handler": self.reset,
            },
        }
        self.command_table = {**self.local_command_table, **command_table}

    def print_commands(self, _):
        """Display all the commands we know about"""
        print(
            "\n".join(
                [
                    f"- {command} {' '.join(info['args'])}: {info['description']}"
                    for command, info in self.command_table.items()
                ]
            )
        )

    def reset(self, _):
        """Reset the state of the commands"""
        self.symbol_table.clear()
        print("State has been reset.")

    def set_variable(self, args):
        """Set a variable to a value. Legal values are words and numbers"""
        if not args:
            print(self.command_info["usage"])
            return False
        else:
            self.symbol_table[args[0]] = args[1]
            print(f"Variable '{args[0]}' has been set to '{args[1]}'.")
            return True

    def show_variable(self, args):
        """Show the value of a variable"""
        if len(args) == 0 and len(self.symbol_table) > 0:
            self.show_all_variables()
        elif len(args) == 1:
            print(f" {self.command_input[1]} -> {args[0]}")
        else:
            print(self.command_info["usage"], "set_variable")

    def get_variable(self, name):
        if len(self.symbol_table) == 0:
            raise ValueError("No variables have been set")
        result = self.symbol_table.get(name, False)
        if result is False:
            raise ValueError("Unknown variable")
        return result

    def show_all_variables(self):
        print(
            "\n".join([f"{name} -> {value}" for name, value in self.symbol_table.items()])
        )

    def get_tokens(self, val_list):
        """parse a list of values, or return False if there's a parse error"""
        result = []
        for val in val_list:
            a_val = self.get_token(val)
            if a_val is False:
                return False
            else:
                result.append(a_val)
        return result

    def get_token(self, token):
        """Parse a value from a token. Either a word or a float or a list"""
        # Check if the token is a variable name, and if so, convert it to its digital value
        if token.isidentifier():
            value = self.symbol_table.get(token)
            if not value:
                value = token
            return value
        try:
            return float(token)
        except ValueError:
            pass
        # if not, see if works as a json value
        try:
            return loads(token)
        except JSONDecodeError:
            print(f"bad json: {token}")
            return False
        return False

    def get_command(self):
        """Prompt for the next command and then parse the command"""
        while True:
            try:
                raw_input = self.session.prompt(">>> ")
                self.command_input = raw_input.strip().split()
                if len(self.command_input) > 0:
                    command = self.command_input[0]
                    args = self.get_tokens(self.command_input[1:])
                    return command, args
                return None, None
            except KeyboardInterrupt:
                continue

    def cli(self):
        """Main entry point to parse commands"""
        while True:
            command, args = self.get_command()
            if command not in self.command_table:
                print("Invalid command. Type 'help' to see the available commands.")
                continue
            # Execute the command
            self.command_info = self.command_table[command]
            self.command_info["handler"](args)
