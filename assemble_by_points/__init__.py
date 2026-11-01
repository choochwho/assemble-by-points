"""Assemble by Points: fit two mesh objects together and merge them into one."""

# Development: F3 > "Reload Scripts" re-runs this file with its old globals
# still present, so reload every submodule (dependencies first) to pick up edits.
if "operators" in locals():
    import importlib
    for _mod in (fitting, merge, properties, operators, ui):  # noqa: F821
        importlib.reload(_mod)

from . import fitting, merge, operators, properties, ui  # noqa: E402

_modules = (properties, operators, ui)


def register():
    for mod in _modules:
        mod.register()


def unregister():
    for mod in reversed(_modules):
        mod.unregister()
