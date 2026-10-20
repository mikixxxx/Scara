"""Optional PySide6 smoke tests (run on user's PC when PySide6 is installed)."""
import json
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import patch

# Useful on headless CI; no change to the real CAD runtime.
os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
try:
    from PySide6.QtWidgets import QApplication
    from scara_cad import MainWindow
    PYSIDE_AVAILABLE = True
except ImportError:
    PYSIDE_AVAILABLE = False


@unittest.skipUnless(PYSIDE_AVAILABLE, "PySide6 is not installed")
class TestToolLibraryUi(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def setUp(self):
        self.window = MainWindow()

    def tearDown(self):
        self.window.close()

    def test_new_tool_and_library_save(self):
        with patch("scara_cad.QInputDialog.getText", return_value=("Kamera", True)):
            self.window.add_tool()
        self.assertEqual(self.window.project["active_tool"], "Kamera")
        self.assertEqual(self.window.tool_combo.currentText(), "Kamera")
        self.assertEqual(len(self.window.project["tools"]), 2)
        with tempfile.TemporaryDirectory() as folder:
            target = Path(folder) / "scara_tools.json"
            self.window._library_file = lambda: target
            with patch("scara_cad.QMessageBox.information"):
                self.window.save_tool_library()
            self.assertTrue(target.exists())
            loaded = json.loads(target.read_text(encoding="utf-8"))
            self.assertEqual(loaded["schema"], 1)
            self.assertEqual(loaded["tools"][-1]["name"], "Kamera")

    def test_repaint_after_create_tool(self):
        with patch("scara_cad.QInputDialog.getText", return_value=("Jehla", True)):
            self.window.add_tool()
        # Forces SketchCanvas.paintEvent() and verifies the grid tool scope.
        self.window.show()
        self.app.processEvents()
        self.assertEqual(self.window.project["active_tool"], "Jehla")


if __name__ == "__main__":
    unittest.main()
