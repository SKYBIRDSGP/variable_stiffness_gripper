import os

_HERE = os.path.dirname(os.path.abspath(__file__))
XML_PATH = os.path.join(_HERE, "panda.xml")
SCENE_PATH = os.path.join(_HERE, "scene.xml")


def build_xml():
    assert os.path.exists(XML_PATH), (
        f"panda.xml not found at {XML_PATH}\n"
        "Run download_assets.py first, or ensure the file exists."
    )
    return XML_PATH


def get_xml_path():
    return XML_PATH


def get_scene_path():
    return SCENE_PATH
