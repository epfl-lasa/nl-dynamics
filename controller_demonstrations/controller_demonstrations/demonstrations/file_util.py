import os

def get_base():
    return os.path.dirname(__file__)

def get_files(base, directory):
    path = os.path.join(base, directory)
    files = os.listdir(path)
    return files

def get_fpath(directory, filename, relative=False):
    base = ''
    if relative:
        base = os.path.dirname(__file__)
    path = os.path.join(base, directory, filename)
    return path
