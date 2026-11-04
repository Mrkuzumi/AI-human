import base64
import hashlib
import hmac
import json
import unittest
from urllib.parse import parse_qs, urlsplit

from stream_receiver import build_asr_url, parse_result


class StreamReceiverTest(unittest.TestCase):
    def test_builds_valid_signature(self):
        url = build_asr_url(
            "123456",
            "secret-id",
            "secret-key",
            800,
            timestamp=1_700_000_000,
            nonce=123,
            voice_id="voice-id",
        )
        parsed = urlsplit(url)
        params = parse_qs(parsed.query)
        self.assertEqual(params["engine_model_type"], ["16k_zh_en_2.0"])
        signature = params.pop("signature")[0]
        query = urlencode_sorted(params)
        target = f"{parsed.netloc}{parsed.path}?{query}"
        expected = base64.b64encode(
            hmac.new(b"secret-key", target.encode(), hashlib.sha1).digest()
        ).decode()
        self.assertEqual(signature, expected)

    def test_parses_partial_and_final_results(self):
        partial = json.dumps(
            {"code": 0, "result": {"slice_type": 1, "voice_text_str": "你好"}}
        )
        final = json.dumps(
            {"code": 0, "result": {"slice_type": 2, "voice_text_str": "你好。"}}
        )
        self.assertEqual(parse_result(partial), ("你好", False))
        self.assertEqual(parse_result(final), ("你好。", True))
        v2 = json.dumps({"code": 0, "sentences": {"sentence_id": 1,
                         "sentence_type": 1, "sentence": "你好。"}})
        self.assertEqual(parse_result(v2), ("你好。", True))


def urlencode_sorted(params):
    from urllib.parse import urlencode

    return urlencode(sorted((key, values[0]) for key, values in params.items()))


if __name__ == "__main__":
    unittest.main()
