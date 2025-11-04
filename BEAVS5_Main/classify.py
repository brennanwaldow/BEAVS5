import tree_sitter as ts
import tree_sitter_cpp as tscpp
import argparse
import os


def get_function(node, name):
    for child in node.children:
        if child.type == "primitive_type":
            type_node = child
        elif child.type == "function_declarator":
            decl_node = child
        elif child.type == "compound_statement":
            stmt_node = child

    decl = b"%s %s;" % (type_node.text, decl_node.text)
    defi = b"%s %s::%s %s" % (type_node.text, name, decl_node.text, stmt_node.text)

    return decl, defi


def read_decls(node, name):
    out_class = []
    in_class = []
    source = []

    preprocs = []

    # This doesn't handle everything
    # It doesn't put anything in out_class even though for example a "using std::atan;" or function declarations
    for child in node.children:
        if child.type == "function_definition":
            decl, defi = get_function(child, name)
            in_class.append(decl)
            source.append(defi)
        elif child.type == "declaration":
            in_class.append(b"%s" % (child.text))
        elif child.type == "enum_specifier":
            in_class.append(b"%s;" % (child.text))
        elif child.type == "preproc_def":
            preprocs.append(b"%s" % (child.text))

    return preprocs + out_class, in_class, source


def make_class(in_class, name, adds):
    body = b"\n  ".join(in_class)
    return b"class %s {\npublic:\n%s\n  %s\n};" % (name, adds, body)


def make_header(out_class, in_class, name, out_adds, in_adds):
    clazz = make_class(in_class, name, in_adds)
    head = b"\n".join(out_class)

    caps_name = "".join(c.upper() for c in name.decode("utf-8")).encode("utf-8")
    return b"#ifndef %s_H\n#define %s_H\n\n%s\n%s\n\n%s\n\n#endif\n" % (
        caps_name,
        caps_name,
        out_adds,
        head,
        clazz,
    )


def make_source(source, header_name, adds):
    defis = b"\n\n".join(source)

    return b'#include "%s"\n\n%s\n%s\n' % (header_name, adds, defis)


# TODO: Maybe add implicit array sizes
def classify_file(in_path, out_path, additions_path, name):
    name = name.encode("utf-8")
    with open(additions_path, "rb") as file:
        text = file.read()
        out_adds, in_adds, source_adds = text.split(b"\n------\n\n")

    with open(in_path, "rb") as file:
        code = file.read()

    parser = ts.Parser(ts.Language(tscpp.language()))

    tree = parser.parse(code)
    root = tree.root_node

    out_class, in_class, source = read_decls(root, name)

    header_path = "%s.h" % out_path
    source_path = "%s.cpp" % out_path

    header_text = make_header(out_class, in_class, name, out_adds, in_adds)
    source_text = make_source(
        source, os.path.basename(header_path).encode("utf-8"), source_adds
    )

    with open(header_path, "wb+") as file:
        file.write(header_text)
    with open(source_path, "wb+") as file:
        file.write(source_text)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()

    parser.add_argument("in_path", type=str, help="The input file path as a string.")

    parser.add_argument(
        "additions_path",
        type=str,
        help="The includes file path as a string.",
    )

    parser.add_argument(
        "out_path",
        type=str,
        help="The output file path (no extension) as a string.",
    )

    parser.add_argument("name", type=str, help="The class name.")

    args = parser.parse_args()
    classify_file(args.in_path, args.out_path, args.additions_path, args.name)
