#! /usr/bin/env python
# -*- coding: utf-8 -*-

import os
import sys
import getopt
import re
import string
import xml.dom
import xml.parsers.expat
from xml.dom.minidom import parse


_FIND_PATTERN = re.compile(r"\$\(\s*find\s+([^\)\s]+)\s*\)")


def _find_package_dir(package_name, base_dir):
    current = os.path.abspath(base_dir)
    while True:
        if os.path.basename(current) == package_name:
            return current
        candidate = os.path.join(current, package_name)
        if os.path.isdir(candidate):
            return candidate
        parent = os.path.dirname(current)
        if parent == current:
            break
        current = parent
    return None


def _expand_find_substitutions(path_value, base_dir):
    def repl(match):
        package_name = match.group(1)
        package_dir = _find_package_dir(package_name, base_dir)
        return package_dir if package_dir else match.group(0)

    return _FIND_PATTERN.sub(repl, path_value)


def _write_data_compat(writer, data, attr=None):
    try:
        # Python 3.12+
        return xml.dom.minidom._write_data(writer, data, attr)
    except TypeError:
        # Python <= 3.11
        return xml.dom.minidom._write_data(writer, data)

# =============================================================================
# Exceptions
# =============================================================================

class XacroException(Exception):
    pass

# =============================================================================
# Utilities
# =============================================================================

def isnumber(x):
    return hasattr(x, '__int__')

# Better pretty printing of xml
def fixed_writexml(self, writer, indent="", addindent="", newl=""):
    writer.write(indent + "<" + self.tagName)

    attrs = self._get_attributes()
    a_names = sorted(attrs.keys())

    for a_name in a_names:
        writer.write(f' {a_name}="')
        _write_data_compat(writer, attrs[a_name].value, None)
        writer.write('"')

    if self.childNodes:
        if len(self.childNodes) == 1 and \
           self.childNodes[0].nodeType == xml.dom.minidom.Node.TEXT_NODE:
            writer.write(">")
            self.childNodes[0].writexml(writer, "", "", "")
            writer.write(f"</{self.tagName}>{newl}")
            return

        writer.write(f">{newl}")
        for node in self.childNodes:
            if node.nodeType != xml.dom.minidom.Node.TEXT_NODE:
                node.writexml(writer, indent + addindent, addindent, newl)
        writer.write(f"{indent}</{self.tagName}>{newl}")
    else:
        writer.write(f"/>{newl}")

xml.dom.minidom.Element.writexml = fixed_writexml

# =============================================================================
# Symbol Table
# =============================================================================

class Table:
    def __init__(self, parent=None):
        self.parent = parent
        self.table = {}

    def __getitem__(self, key):
        if key in self.table:
            return self.table[key]
        if self.parent:
            return self.parent[key]
        raise KeyError(key)

    def __setitem__(self, key, value):
        self.table[key] = value

    def __contains__(self, key):
        return key in self.table or (self.parent and key in self.parent)

# =============================================================================
# Lexer
# =============================================================================

class QuickLexer:
    def __init__(self, **res):
        self.str = ""
        self.top = None
        self.res = []
        for k, v in res.items():
            setattr(self, k, len(self.res))
            self.res.append(v)

    def lex(self, s):
        self.str = s
        self.top = None
        self.next()

    def peek(self):
        return self.top

    def next(self):
        result = self.top
        self.top = None
        for i, r in enumerate(self.res):
            m = re.match(r, self.str)
            if m:
                self.top = (i, m.group(0))
                self.str = self.str[m.end():]
                break
        return result

# =============================================================================
# XML traversal helpers
# =============================================================================

def first_child_element(elt):
    c = elt.firstChild
    while c:
        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
            return c
        c = c.nextSibling
    return None

def next_sibling_element(elt):
    c = elt.nextSibling
    while c:
        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
            return c
        c = c.nextSibling
    return None

def next_element(elt):
    child = first_child_element(elt)
    if child:
        return child
    while elt:
        nxt = next_sibling_element(elt)
        if nxt:
            return nxt
        elt = elt.parentNode
    return None

def next_node(node):
    if node.firstChild:
        return node.firstChild
    while node:
        if node.nextSibling:
            return node.nextSibling
        node = node.parentNode
    return None

def child_elements(elt):
    c = elt.firstChild
    while c:
        if c.nodeType == xml.dom.Node.ELEMENT_NODE:
            yield c
        c = c.nextSibling

# =============================================================================
# Xacro core
# =============================================================================

all_includes = []

def process_includes(doc, base_dir):
    namespaces = {}
    previous = doc.documentElement
    elt = next_element(previous)

    while elt:
        if elt.tagName in ("include", "xacro:include"):
            filename = eval_text(elt.getAttribute("filename"), {})
            filename = _expand_find_substitutions(filename, base_dir)
            if not os.path.isabs(filename):
                filename = os.path.join(base_dir, filename)

            try:
                with open(filename, "r", encoding="utf-8") as f:
                    included = parse(f)
                    all_includes.append(filename)
            except Exception as e:
                raise XacroException(f'Failed to include "{filename}": {e}')

            for c in child_elements(included.documentElement):
                elt.parentNode.insertBefore(c.cloneNode(True), elt)
            elt.parentNode.removeChild(elt)
            elt = None

            for name, value in included.documentElement.attributes.items():
                if name.startswith("xmlns:"):
                    namespaces[name] = value
        else:
            previous = elt

        elt = next_element(previous)

    for k, v in namespaces.items():
        doc.documentElement.setAttribute(k, v)

def grab_macros(doc):
    macros = {}
    previous = doc.documentElement
    elt = next_element(previous)

    while elt:
        if elt.tagName in ("macro", "xacro:macro"):
            name = elt.getAttribute("name")
            macros[name] = elt
            macros["xacro:" + name] = elt
            elt.parentNode.removeChild(elt)
            elt = None
        else:
            previous = elt
        elt = next_element(previous)
    return macros

def grab_properties(doc):
    table = Table()
    previous = doc.documentElement
    elt = next_element(previous)

    while elt:
        if elt.tagName in ("property", "xacro:property"):
            name = elt.getAttribute("name")
            value = elt.getAttribute("value") if elt.hasAttribute("value") else elt
            table[name] = value
            elt.parentNode.removeChild(elt)
            elt = None
        else:
            previous = elt
        elt = next_element(previous)
    return table

# =============================================================================
# Expression evaluation
# =============================================================================

def eval_text(text, symbols):
    def handle_expr(s):
        lex = QuickLexer(
            IGNORE=r"\s+",
            NUMBER=r"(\d+(\.\d*)?|\.\d+)([eE][-+]?\d+)?",
            SYMBOL=r"[a-zA-Z_]\w*",
            OP=r"[\+\-\*/]",
            LPAREN=r"\(",
            RPAREN=r"\)",
        )
        lex.lex(s)
        return eval_expr(lex, symbols)

    results = []
    lex = QuickLexer(
        EXPR=r"\$\{[^\}]*\}",
        TEXT=r"([^\$]|\$[^{])+"
    )
    lex.lex(text)

    while lex.peek():
        if lex.peek()[0] == lex.EXPR:
            results.append(str(handle_expr(lex.next()[1][2:-1])))
        else:
            results.append(lex.next()[1])

    return "".join(results)

def eval_expr(lex, symbols):
    result = 0
    if not lex.peek():
        return result
    if lex.peek()[0] in (lex.NUMBER, lex.SYMBOL):
        token = lex.next()[1]
        try:
            return float(token)
        except ValueError:
            value = symbols[token]
            try:
                return float(value)
            except (ValueError, TypeError):
                return str(value)
    return result

def eval_all(root, macros, symbols):
    for at in root.attributes.items():
        root.setAttribute(at[0], eval_text(at[1], symbols))

    previous = root
    node = next_node(previous)

    while node:
        if node.nodeType == xml.dom.Node.ELEMENT_NODE:
            for at in node.attributes.items():
                node.setAttribute(at[0], eval_text(at[1], symbols))
            previous = node
        elif node.nodeType == xml.dom.Node.TEXT_NODE:
            node.data = eval_text(node.data, symbols)
            previous = node
        else:
            previous = node
        node = next_node(previous)

def eval_self_contained(doc):
    macros = grab_macros(doc)
    symbols = grab_properties(doc)
    eval_all(doc.documentElement, macros, symbols)

# =============================================================================
# Public API
# =============================================================================

def convert_xacro_to_urdf(xacro_file: str, output_urdf_path: str) -> None:
    xacro_file = os.path.abspath(xacro_file)
    output_urdf_path = os.path.abspath(output_urdf_path)

    if not os.path.isfile(xacro_file):
        raise FileNotFoundError(xacro_file)

    os.makedirs(os.path.dirname(output_urdf_path), exist_ok=True)

    with open(xacro_file, "r", encoding="utf-8") as f:
        doc = parse(f)

    all_includes.clear()
    process_includes(doc, os.path.dirname(xacro_file))
    eval_self_contained(doc)

    banner = [xml.dom.minidom.Comment(c) for c in [
        "=" * 83,
        f" Autogenerated from {os.path.basename(xacro_file)} ",
        " EDITING THIS FILE BY HAND IS NOT RECOMMENDED ",
        "=" * 83
    ]]

    for c in reversed(banner):
        doc.insertBefore(c, doc.firstChild)

    with open(output_urdf_path, "w", encoding="utf-8") as f:
        f.write(doc.toprettyxml(indent="  "))
        f.write("\n")

# =============================================================================
# CLI
# =============================================================================

def print_usage(code=0):
    print("Usage: xacro_converter.py [-o output.urdf] input.xacro")
    sys.exit(code)

def main():
    try:
        opts, args = getopt.gnu_getopt(sys.argv[1:], "ho:", [])
    except getopt.GetoptError:
        print_usage(2)

    output = None
    for o, a in opts:
        if o == "-h":
            print_usage(0)
        elif o == "-o":
            output = a

    if not args:
        print_usage(2)

    xacro = args[0]
    if output is None:
        output = os.path.splitext(xacro)[0] + ".urdf"

    convert_xacro_to_urdf(xacro, output)

if __name__ == "__main__":
    main()
