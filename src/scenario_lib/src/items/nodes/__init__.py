import os
import sys
import pkgutil
import inspect

from diagramNode import DiagramNode

classes = []

def getAllDiagramNodesClasses(dirname):
    if len(classes) <= 0:
        for importer, package_name, _ in pkgutil.iter_modules([os.path.split(__file__)[0]]):
            full_package_name = '%s.%s' % (dirname, package_name)
            if full_package_name not in sys.modules:
                module = importer.find_module(package_name).load_module(full_package_name)
                for name, obj in inspect.getmembers(module, inspect.isclass):
                    if os.path.dirname(sys.modules[obj.__module__].__file__).endswith(dirname) and name != "DiagramNode":
                        classes.append(obj)
    
    return classes