#!/usr/bin/env python3

import glob
import re
from collections import namedtuple
from os import path, getenv, walk
import lxml.etree as ET
import graphviz

# if PAPARAZZI_HOME not set, then assume the tree containing this
# file is a reasonable substitute
PAPARAZZI_SRC   = getenv("PAPARAZZI_HOME", path.normpath(path.join(path.dirname(path.abspath(__file__)), '../../')))
PAPARAZZI_HOME  = getenv("PAPARAZZI_HOME", PAPARAZZI_SRC)

# Directories
conf_dir        = path.join(PAPARAZZI_HOME, "conf/")
modules_dir     = path.join(conf_dir, "modules/")

PprzModule = namedtuple("PprzModule", "depends conflicts provides recommends suggests")

def get_list_of_files(directory, extension):
    mylist = glob.glob(path.join(directory, "*" + extension))
    mylist.sort()
    ret = []
    for it in mylist:
        ret.append( it.replace(directory, "").replace(extension, ""))
    return ret

def get_list_of_modules():
    return get_list_of_files( modules_dir, ".xml")

def find_or_empty(node, key):
    el = node.find(key)
    if el is not None:
        return re.split('[,|]',el.text)
    else:
        return []

def get_module_dependency(module_name):
    try:
        xml = ET.parse(path.join(modules_dir, module_name + ".xml"))
        root = xml.getroot().find("dep")
        if root is not None:
            lst_depends = find_or_empty(root, 'depends')
            lst_conflicts = find_or_empty(root, 'conflicts')
            lst_provides = find_or_empty(root, 'provides')
            lst_recommends = find_or_empty(root, 'recommends')
            lst_suggests = find_or_empty(root, 'suggests')
            return PprzModule(depends=lst_depends, conflicts=lst_conflicts, provides=lst_provides, recommends=lst_recommends, suggests=lst_suggests)
    except (IOError, ET.XMLSyntaxError) as e:
        print(e.__str__())
    except Exception as e:
        print(e.__str__())

    return PprzModule(depends=[], conflicts=[], provides=[], recommends=[], suggests=[])

if __name__ == '__main__':
    g = graphviz.Digraph('Modules dependency', engine='fdp')

    modules = get_list_of_modules()
    mod_dep = []
    func = []
    edges_m_req = []
    edges_f_req = []
    edges_provide = []
    edges_reco = []
    for m in modules:
        #print('Module '+m)
        dep = get_module_dependency(m)
        if len(dep.depends) > 0:
            mod_dep.append(m)
        else:
            print("Modules without dep: ", m)
        for p in dep.provides:
            if p not in func:
                func.append(p)
            edges_provide.append((m, p))
        for d in dep.depends:
            if d[0] == '@':
                if (d[1:] not in func) and (len(d[1:])>0):
                    func.append(d[1:])
                #else:
                #    print("no add",d)
                edges_f_req.append((m, d[1:]))
            else:
                if d in modules:
                    edges_m_req.append((m, d))
                else:
                    print("not a valid module", m, d)
        for r in dep.recommends:
            if r[0] == '@':
                edges_reco.append((m, r[1:]))
            else:
                if r in modules:
                    edges_reco.append((m, r))
                else:
                    print("not a valid recommends", m, r)


    g.attr('node', shape='doublecircle')
    for f in func:
        g.node(f)
    g.attr('node', shape='circle')
    for m in mod_dep:
        g.node(m)
    for e in edges_m_req:
        g.edge(e[0], e[1])
    g.attr('edge', color='blue')
    for e in edges_f_req:
        g.edge(e[0], e[1])
    g.attr('edge', color='red')
    for e in edges_provide:
        g.edge(e[0], e[1])
    g.attr('edge', color='green')
    for e in edges_reco:
        g.edge(e[0], e[1])

    g.view()

