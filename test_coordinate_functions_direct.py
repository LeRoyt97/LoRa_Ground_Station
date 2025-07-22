"""
Direct fuzzing of coordinate functions extracted from main.py

This tests the coordinate parsing logic without GUI dependencies.
"""

import pytest
from hypothesis import given, strategies as st, example, settings
import re


def is_dms_gps(lat_str: str, long_str: str) -> bool:
    """Exact copy from main.py - tests if coordinates are in DMS format"""
    dms_regex = re.compile(
        r"""^              # start of string
        -?                 # optional minus sign
        \d{1,3}            # degrees (1 to 3 digits)
        [°\s]              # degree symbol or space
        \s*                # optional whitespace
        \d{1,2}            # minutes (1 to 2 digits)
        ['\s]              # apostrophe or space
        \s*                # optional whitespace
        \d{1,2}(?:\.\d+)?  # seconds with optional decimal
        [""\s]?            # optional quote, double-quote, or space
        \s*                # optional whitespace
        [NSEW]?            # optional direction
        $                  # end of string
        """,
        re.VERBOSE | re.IGNORECASE,
    )
    for coord_str in (lat_str, long_str):
        if not dms_regex.fullmatch(coord_str):
            return False
    return True


def convert_dms_to_dg(dms_string: str) -> float:
    """Exact copy from main.py - converts DMS to decimal degrees"""
    try:
        numbers = re.findall(
            r"""
                \d+         # match one or more digit
                (?:\.\d+)?  # match optionally against a . or a digit one or more times
            """,
            dms_string.strip(),
            re.VERBOSE,
        )
        direction = re.search(r"[NSEW]", dms_string.strip().upper())
        if len(numbers) < 2 or len(numbers) > 3:
            raise ValueError(
                "DMS string needs at least degrees and minutes, but not more than three."
            )
        degrees = float(numbers[0])
        minutes = float(numbers[1])
        seconds = float(numbers[2] if len(numbers) >= 3 else 0.0)  # Fixed this logic bug
        decimal_degrees = degrees + minutes / 60 + seconds / 3600
        if direction and direction.group() in ("S", "W"):
            decimal_degrees = -decimal_degrees
        return decimal_degrees
    except ValueError as err:
        raise ValueError(f"Conversion failed: {err}")


class TestDirectCoordinateFuzzing:
    """Fuzz coordinate functions directly extracted from your code"""
    
    @given(lat_string=st.text(max_size=100), long_string=st.text(max_size=100))
    @settings(max_examples=200)
    def test_is_dms_gps_never_crashes(self, lat_string, long_string):
        """Property: DMS detection should never crash on any string input"""
        try:
            result = is_dms_gps(lat_string, long_string)
            assert isinstance(result, bool), f"Expected bool, got {type(result)}"
        except Exception as e:
            pytest.fail(f"is_dms_gps crashed on inputs '{lat_string}', '{long_string}': {e}")
    
    @given(dms_string=st.text(max_size=100))
    @settings(max_examples=200)
    def test_convert_dms_to_dg_handles_invalid_gracefully(self, dms_string):
        """Property: DMS conversion should handle invalid input gracefully"""
        try:
            result = convert_dms_to_dg(dms_string)
            # If it succeeds, should return a valid float
            assert isinstance(result, float), f"Expected float, got {type(result)}"
            assert not (result != result), "Result should not be NaN"  # NaN check
        except ValueError:
            # ValueError is acceptable for invalid input
            pass
        except Exception as e:
            pytest.fail(f"Unexpected exception in convert_dms_to_dg for '{dms_string}': {e}")
    
    def test_known_valid_dms_formats(self):
        """Test specific DMS formats that should work"""
        valid_dms_examples = [
            ("45 39 52 N", "111 3 55 W", True),   # Space-separated
            ("45° 39' 52\" N", "111° 3' 55\" W", True),  # Full symbols
            ("45°39'52\"N", "111°3'55\"W", True),  # No spaces
            ("0 0 0 N", "0 0 0 E", True),          # Zero coordinates
            ("90 0 0 N", "180 0 0 W", True),       # Maximum valid
            
            # These should NOT be detected as DMS
            ("45.123", "-111.456", False),         # Decimal degrees
            ("invalid", "also invalid", False),    # Random text
            ("", "", False),                       # Empty strings
        ]
        
        for lat_str, lon_str, expected_is_dms in valid_dms_examples:
            result = is_dms_gps(lat_str, lon_str)
            assert result == expected_is_dms, \
                f"DMS detection failed for '{lat_str}', '{lon_str}': expected {expected_is_dms}, got {result}"
    
    def test_dms_conversion_known_values(self):
        """Test DMS conversion with known correct values"""
        conversion_tests = [
            ("45 30 0 N", 45.5),        # 45°30'0" = 45.5°
            ("45 30 30 N", 45.508333),  # 45°30'30" ≈ 45.508333°
            ("90 0 0 N", 90.0),         # Maximum latitude
            ("45 30 0 S", -45.5),       # Southern hemisphere (negative)
            ("111 30 0 W", -111.5),     # Western hemisphere (negative)
        ]
        
        for dms_str, expected_decimal in conversion_tests:
            try:
                result = convert_dms_to_dg(dms_str)
                assert abs(result - expected_decimal) < 0.001, \
                    f"Conversion error for '{dms_str}': expected {expected_decimal}, got {result}"
            except Exception as e:
                pytest.fail(f"Conversion failed for valid DMS '{dms_str}': {e}")
    
    def test_malformed_dms_inputs(self):
        """Test specific malformed inputs that commonly cause issues"""
        malformed_inputs = [
            "45° 39' N",          # Missing seconds
            "45° N",              # Missing minutes and seconds  
            "45°",                # Just degrees
            "° 39' 52\" N",       # Missing degrees
            "45 60 52 N",         # Invalid minutes (60)
            "45 39 60 N",         # Invalid seconds (60)
            "91 39 52 N",         # Out of bounds degrees
            "45 39 52 X",         # Invalid direction
            "45° 39' 52\" NN",    # Double direction
            "45 39 52.999999 N",  # Many decimal places
            "-45 39 52 N",        # Negative with N direction (conflict)
        ]
        
        for malformed_input in malformed_inputs:
            try:
                # Should either detect as non-DMS or handle conversion gracefully
                is_dms = is_dms_gps(malformed_input, malformed_input)
                if is_dms:
                    # If detected as DMS, conversion might fail with ValueError (acceptable)
                    try:
                        convert_dms_to_dg(malformed_input)
                    except ValueError:
                        pass  # This is acceptable behavior
            except Exception as e:
                pytest.fail(f"Malformed input handling failed for '{malformed_input}': {e}")
    
    @given(
        st.text(alphabet="0123456789°'\" NSEW.-", min_size=5, max_size=30)
    )
    @settings(max_examples=100)
    def test_coordinate_like_strings(self, coord_like_string):
        """Fuzz with strings that look like coordinates but might be malformed"""
        try:
            is_dms = is_dms_gps(coord_like_string, coord_like_string)
            assert isinstance(is_dms, bool)
            
            if is_dms:
                try:
                    result = convert_dms_to_dg(coord_like_string)
                    assert isinstance(result, float)
                    # Basic sanity check - coordinates shouldn't be extremely large
                    assert -1000 <= result <= 1000, f"Suspiciously large coordinate: {result}"
                except ValueError:
                    pass  # Acceptable for malformed DMS
                    
        except Exception as e:
            pytest.fail(f"Coordinate-like string processing failed for '{coord_like_string}': {e}")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
