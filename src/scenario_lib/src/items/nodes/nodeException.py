#!/usr/bin/env python
# -*- coding: utf-8 -*-

class NodeException(Exception):
    def __init__(self, nodeCausingErrror, message):
        self.nodeCausingErrror = nodeCausingErrror
        super(NodeException, self).__init__(message)