"""Test cases for rg"""

from rgparser2 import Parser

def test_get_token():
    parser = Parser({"y": 100}, {})
    assert parser.get_token("1.0") == 1.0
    assert parser.get_token("") is False
    assert parser.get_token("x") == "x"
    assert parser.get_token("y") == 100.0
    assert parser.get_token("[[1,2],[1,2]]") == [[1,2],[1,2]]

def test_get_tokens():
    parser = Parser({"y": 100}, {})
    assert parser.get_tokens(["1.0", "x"]) == [1.0, "x"]
    assert parser.get_tokens("") == []
    assert parser.get_tokens(["x", "y"]) == ["x", 100]
    assert parser.get_tokens([]) == []

def test_set_variable():
    parser = Parser({"y": 100}, {})
    parser.set_variable(["aa", 100])
    assert parser.get_variable("aa") == 100
    