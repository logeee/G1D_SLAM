import io
import json
import unittest
from unittest.mock import patch

from backend.app.arm_targets import ArmTargetCatalog


class _Response(io.BytesIO):
    def __enter__(self):
        return self

    def __exit__(self, *_args):
        self.close()


def _response(payload):
    return _Response(json.dumps(payload).encode("utf-8"))


class ArmTargetCatalogTests(unittest.TestCase):
    def test_joins_model_labels_with_cigarette_names(self):
        replies = [
            _response({"effective_model": "YanHe20class.pt", "yolo_class_names": ["31019915", "43010159"]}),
            _response({
                "object_infos": {
                    "31019915": {"name": "熊猫(典藏中支)", "length_mm": 160, "width_mm": 90, "height_mm": 20},
                    "43010159": {"name": "白沙(和天下尊品中支)"},
                }
            }),
        ]
        with patch("backend.app.arm_targets.urllib.request.urlopen", side_effect=replies):
            result = ArmTargetCatalog("http://127.0.0.1:18081").read()

        self.assertTrue(result["ok"])
        self.assertEqual("YanHe20class.pt", result["model"])
        self.assertEqual("31019915", result["targets"][0]["target_object"])
        self.assertEqual("熊猫(典藏中支) (31019915)", result["targets"][0]["display_name"])
        self.assertEqual(160.0, result["targets"][0]["length_mm"])

    def test_keeps_model_targets_when_object_information_is_unavailable(self):
        with patch(
            "backend.app.arm_targets.urllib.request.urlopen",
            side_effect=[_response({"effective_model": "model.pt", "yolo_class_names": ["label-a"]}), OSError("offline")],
        ):
            result = ArmTargetCatalog("http://127.0.0.1:18081").read()

        self.assertTrue(result["ok"])
        self.assertEqual("label-a", result["targets"][0]["display_name"])
        self.assertTrue(result["warnings"])

    def test_model_catalog_failure_is_reported(self):
        with patch("backend.app.arm_targets.urllib.request.urlopen", side_effect=OSError("connection refused")):
            result = ArmTargetCatalog("http://127.0.0.1:18081").read()

        self.assertFalse(result["ok"])
        self.assertEqual([], result["targets"])
        self.assertIn("connection refused", result["error"])


if __name__ == "__main__":
    unittest.main()
