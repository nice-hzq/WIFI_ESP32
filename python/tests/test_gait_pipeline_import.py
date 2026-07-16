# -*- coding: utf-8 -*-
import importlib
import unittest


class GaitPipelineImportTests(unittest.TestCase):
    def test_optional_attitude_plotter_does_not_break_import(self):
        module = importlib.import_module("gait.gait_pipeline")
        self.assertTrue(callable(module.run_gait_pipeline))


if __name__ == "__main__":
    unittest.main()
