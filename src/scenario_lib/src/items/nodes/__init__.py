import os
import sys
import pkgutil
import inspect
import warnings

from diagramNode import DiagramNode

classes = []

def getAllDiagramNodesClasses(dirname):
    warnings.simplefilter("ignore")
    
    if len(classes) <= 0:
        for importer, package_name, _ in pkgutil.iter_modules([os.path.split(__file__)[0]]):
            full_package_name = '%s.%s' % (dirname, package_name)
            if full_package_name not in sys.modules:
                module = importer.find_module(package_name).load_module(full_package_name)
                for name, obj in inspect.getmembers(module, inspect.isclass):
                    if issubclass(obj, DiagramNode) and name != "DiagramNode":
                        if obj.__name__ not in [classesItem.__name__ for classesItem in classes]:
                            classes.append(obj)
    
    warnings.simplefilter("default")
    
    return classes