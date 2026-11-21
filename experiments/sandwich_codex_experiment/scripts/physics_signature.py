"""Fingerprint the complete SDF except rendering-only visual elements."""
import hashlib
import xml.etree.ElementTree as ET

def physics_signature(path):
    root=ET.parse(path).getroot()
    for parent in root.iter():
        for child in list(parent):
            if child.tag=='visual':parent.remove(child)
    for element in root.iter():
        element.text=element.text.strip() if element.text else None
        element.tail=None
    return hashlib.sha256(ET.tostring(root)).hexdigest()
