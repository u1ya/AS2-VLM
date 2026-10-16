import base64
import io
import tempfile
import unittest
from pathlib import Path

from as2_vlm.vlm import VLMError, read_image


class ImagePreparationTests(unittest.TestCase):
    def setUp(self):
        try:
            from PIL import Image
            self.Image = Image
        except ImportError:
            self.skipTest("Pillow vision extra is not installed")

    def test_large_frame_resized_without_changing_aspect_ratio(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "frame.png"
            self.Image.new("RGB", (1600, 1200)).save(path)
            prepared = read_image(path)
            with self.Image.open(io.BytesIO(base64.b64decode(prepared))) as image:
                self.assertEqual(image.size, (768, 576))
                self.assertEqual(image.format, "JPEG")

    def test_invalid_file_rejected(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "bad.png"
            path.write_bytes(b"\x89PNG\r\n\x1a\nnot an image")
            with self.assertRaises(VLMError):
                read_image(path)
