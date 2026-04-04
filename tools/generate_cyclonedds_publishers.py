#!/usr/bin/env python3
"""
Generate CycloneDDS publisher/subscriber .cpp files from FastDDS sources.

Reads each .cpp file from dds/fastdds/publishers/ and dds/fastdds/subscribers/,
applies conversion rules to transform FastDDS C++ accessor patterns into
CycloneDDS C struct field access, and writes the output to
dds/cyclonedds/publishers/ and dds/cyclonedds/subscribers/.

Files that need full manual creation are skipped.
"""

import os
import re
import sys
from pathlib import Path

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

# Files that need full manual creation -- skip entirely
SKIP_FILES = {
    "AutowarePublisher.cpp",
    "AutowarePublisherBase.hpp",
    "AutowareSubscriber.cpp",
    "AutowareController.cpp",
}

# Base directory (relative to this script or absolute)
SCRIPT_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = SCRIPT_DIR.parent
DDS_BASE = PROJECT_ROOT / "LibCarla" / "source" / "carla" / "ros2" / "dds"

FASTDDS_DIRS = [
    ("publishers",  DDS_BASE / "fastdds" / "publishers"),
    ("subscribers", DDS_BASE / "fastdds" / "subscribers"),
]

CYCLONE_DIRS = {
    "publishers":  DDS_BASE / "cyclonedds" / "publishers",
    "subscribers": DDS_BASE / "cyclonedds" / "subscribers",
}

# ---------------------------------------------------------------------------
# A. Type name mapping  (order matters -- longer/more-specific first)
# ---------------------------------------------------------------------------

# Explicit mapping from C++ namespace type to CycloneDDS C typedef name.
# Listed longest-first to avoid partial replacement issues.
TYPE_MAP = [
    ("geometry_msgs::msg::PoseWithCovarianceStamped", "geometry_msgs_msg_PoseWithCovarianceStamped"),
    ("geometry_msgs::msg::PoseWithCovariance",        "geometry_msgs_msg_PoseWithCovariance"),
    ("geometry_msgs::msg::TwistWithCovariance",       "geometry_msgs_msg_TwistWithCovariance"),
    ("geometry_msgs::msg::TransformStamped",          "geometry_msgs_msg_TransformStamped"),
    ("geometry_msgs::msg::PoseStamped",               "geometry_msgs_msg_PoseStamped"),
    ("geometry_msgs::msg::Transform",                 "geometry_msgs_msg_Transform"),
    ("geometry_msgs::msg::Quaternion",                "geometry_msgs_msg_Quaternion"),
    ("geometry_msgs::msg::Vector3",                   "geometry_msgs_msg_Vector3"),
    ("geometry_msgs::msg::Point",                     "geometry_msgs_msg_Point"),
    ("geometry_msgs::msg::Pose",                      "geometry_msgs_msg_Pose"),
    ("geometry_msgs::msg::Twist",                     "geometry_msgs_msg_Twist"),
    ("sensor_msgs::msg::PointCloud2",                 "sensor_msgs_msg_PointCloud2"),
    ("sensor_msgs::msg::PointField",                  "sensor_msgs_msg_PointField"),
    ("sensor_msgs::msg::CameraInfo",                  "sensor_msgs_msg_CameraInfo"),
    ("sensor_msgs::msg::NavSatStatus",                "sensor_msgs_msg_NavSatStatus"),
    ("sensor_msgs::msg::NavSatFix",                   "sensor_msgs_msg_NavSatFix"),
    ("sensor_msgs::msg::RegionOfInterest",            "sensor_msgs_msg_RegionOfInterest"),
    ("sensor_msgs::msg::Image",                       "sensor_msgs_msg_Image"),
    ("sensor_msgs::msg::Imu",                         "sensor_msgs_msg_Imu"),
    ("builtin_interfaces::msg::Time",                 "builtin_interfaces_msg_Time"),
    ("std_msgs::msg::Header",                         "std_msgs_msg_Header"),
    ("std_msgs::msg::String",                         "std_msgs_msg_String_"),
    ("std_msgs::msg::Float32",                        "std_msgs_msg_Float32"),
    ("tf2_msgs::msg::TFMessage",                      "tf2_msgs_msg_TFMessage"),
    ("nav_msgs::msg::Odometry",                       "nav_msgs_msg_Odometry"),
    ("rosgraph::msg::Clock",                          "rosgraph_msgs_msg_Clock"),
    ("carla_msgs::msg::CarlaEgoVehicleControl",       "carla_msgs_msg_CarlaEgoVehicleControl"),
    ("carla_msgs::msg::CarlaCollisionEvent",          "carla_msgs_msg_CarlaCollisionEvent"),
    ("carla_msgs::msg::CarlaLineInvasion",            "carla_msgs_msg_CarlaLineInvasion"),
    ("carla_msgs::msg::LaneInvasionEvent",            "carla_msgs_msg_LaneInvasionEvent"),
]

# Fallback pattern: pkg::msg::Type -> pkg_msg_Type  (with rosgraph special case)
# Applied after explicit map above.

# ---------------------------------------------------------------------------
# E. PointField constants -> integer values
# ---------------------------------------------------------------------------

POINTFIELD_CONSTANTS = {
    "sensor_msgs::msg::PointField__INT8":     "1",
    "sensor_msgs::msg::PointField__UINT8":    "2",
    "sensor_msgs::msg::PointField__INT16":    "3",
    "sensor_msgs::msg::PointField__UINT16":   "4",
    "sensor_msgs::msg::PointField__INT32":    "5",
    "sensor_msgs::msg::PointField__UINT32":   "6",
    "sensor_msgs::msg::PointField__FLOAT32":  "7",
    "sensor_msgs::msg::PointField__FLOAT64":  "8",
}

# After type mapping, the constants become e.g. sensor_msgs_msg_PointField__FLOAT32
# We also need to handle the mapped form.
POINTFIELD_CONSTANTS_MAPPED = {
    "sensor_msgs_msg_PointField__INT8":     "1",
    "sensor_msgs_msg_PointField__UINT8":    "2",
    "sensor_msgs_msg_PointField__INT16":    "3",
    "sensor_msgs_msg_PointField__UINT16":   "4",
    "sensor_msgs_msg_PointField__INT32":    "5",
    "sensor_msgs_msg_PointField__UINT32":   "6",
    "sensor_msgs_msg_PointField__FLOAT32":  "7",
    "sensor_msgs_msg_PointField__FLOAT64":  "8",
}

# ---------------------------------------------------------------------------
# Methods that should NOT be converted from .method(arg) to .method = arg
# ---------------------------------------------------------------------------

# Known class/API method names that are real function calls (not setters)
SKIP_METHODS = {
    # DDS / publisher / subscriber API
    "Init", "Write", "Publish", "PublishImage", "PublishInfo", "PublishPointCloud",
    "SetData", "SetDataEx", "SetImageData", "SetCameraInfoData", "SetPointCloudData",
    "SetInfoRegionOfInterest", "SetOnDataCallback", "TakeNextSample",
    "IsConnected", "Destroy", "DestroySubscriber",
    "CreateDDSPublisher", "CreateDDSSubscriber",
    "ValidTopicName", "ForwardMessage", "HasBeenInitialized",
    "InitImage", "InitInfo", "InitPointCloud", "InitInfoData",
    "ConvertToRosFormat", "Read", "GetMessage", "IsAlive", "HasNewMessage",
    "GetVehicle", "GetActor",
    # STL / standard library methods
    "emplace_back", "reserve", "resize", "push_back",
    "at", "c_str", "size", "empty", "value",
    "begin", "end", "find", "substr", "append", "insert", "erase",
    "memcpy", "memcmp", "memset",
    "make_shared", "make_unique", "move",
    "cosf", "sinf", "atan2", "hypot", "sqrt",
    "str", "string",
    # Constructors that look like Type(...)
    "std::string", "std::vector", "std::array", "std::ostringstream",
    "std::runtime_error", "std::move",
}

# Additional patterns: anything that starts with uppercase and looks like
# a constructor or free function, e.g. TopicConfig(...), Path(...)
# We handle these via heuristic: skip if the "method" name starts with uppercase
# and is preceded by space/= (constructor context), or is in SKIP_METHODS.

# ---------------------------------------------------------------------------
# Conversion functions
# ---------------------------------------------------------------------------

def apply_type_mapping(line: str) -> str:
    """A. Replace C++ namespace types with CycloneDDS C typedef names.

    String literals (quoted text) are excluded from replacement so that
    type-registry keys like CreateDDSPublisher("sensor_msgs::msg::Image")
    keep their ROS2-format names.
    """
    # Split line into segments: outside-quotes and inside-quotes
    # We only apply type mapping to outside-quotes segments.
    parts = re.split(r'("(?:[^"\\]|\\.)*")', line)
    for i, part in enumerate(parts):
        if i % 2 == 0:  # outside quotes
            for cpp_type, c_type in TYPE_MAP:
                part = part.replace(cpp_type, c_type)

            # Fallback: catch any remaining pkg::msg::Type patterns
            def _fallback_replace(m):
                pkg = m.group(1)
                typ = m.group(2)
                # Special case: rosgraph -> rosgraph_msgs
                if pkg == "rosgraph":
                    pkg = "rosgraph_msgs"
                return f"{pkg}_msg_{typ}"

            part = re.sub(r'(\w+)::msg::(\w+)', _fallback_replace, part)
            parts[i] = part
    return ''.join(parts)


def remove_pubsubtypes_include(line: str) -> bool:
    """B. Return True if this line is a PubSubTypes.h include that should be removed."""
    return bool(re.match(r'\s*#include\s+".*PubSubTypes\.h"', line))


def is_skip_method(method_name: str) -> bool:
    """Check if a method name should NOT be converted to field assignment."""
    if method_name in SKIP_METHODS:
        return True
    # Skip anything that looks like a type constructor: starts with uppercase
    # and contains underscore pattern typical of mapped types, e.g. sensor_msgs_msg_CameraInfo
    if re.match(r'^[A-Z]', method_name):
        return True
    return False


def convert_pointfield_constants(line: str) -> str:
    """E. Replace PointField enum constants with integer values."""
    # First handle the original C++ namespace form
    for const, val in POINTFIELD_CONSTANTS.items():
        line = line.replace(const, val)
    # Then handle the already-mapped form (in case type mapping ran first)
    for const, val in POINTFIELD_CONSTANTS_MAPPED.items():
        line = line.replace(const, val)
    return line


def convert_setter_to_assignment(line: str) -> str:
    """C. Convert .field(value); patterns to .field = value;

    Handles:
      obj.field(value);          -> obj.field = value;
      obj.field(std::move(val))  -> obj.field = val
    Does NOT convert method calls from SKIP_METHODS or STL methods.
    """
    # Pattern: .identifier(args);  at statement level
    # We need to be careful not to match:
    #  - Chained calls like .foo().bar(val);  (handled separately in D)
    #  - Constructor calls like Type name(args);
    #  - Function calls like func(args);

    # Strategy: Find lines with pattern  <prefix>.method(<args>); or <prefix>.method(<args>)
    # where method is NOT in skip list.

    # Match: something.method(args);
    # But NOT: something.method().other(args);  (that's a chained getter+setter)
    # Also need to handle: _impl->_event.field(value);

    # First, handle std::move wrapping in setter context:
    # .field(std::move(value))  ->  .field = value
    # This is specifically for struct field assignment, not for function args

    result = line

    # Pattern for single setter at end of statement:
    # <lhs>.method(arg);
    # where <lhs> does NOT end with () (which would indicate a getter chain)
    # and method is not in skip list

    # Regex explanation:
    #   (.*?)           - prefix (non-greedy)
    #   \.              - dot
    #   (\w+)           - method name
    #   \(              - open paren
    #   (.+)            - arguments
    #   \)              - close paren
    #   \s*;            - semicolon (possibly with trailing whitespace)

    # We do NOT want to match lines like:
    #   if (!_impl->_dds->Init(config, _name, ...))
    #   _impl->_dds = CreateDDSPublisher("sensor_msgs::msg::Image");
    # These have multiple args or are not setters.

    # Simple single-arg setter pattern: .field(single_arg);
    m = re.match(
        r'^(\s*(?:.*?))'      # leading whitespace + prefix (group 1)
        r'\.'                  # dot before method
        r'(\w+)'              # method name (group 2)
        r'\('                  # open paren
        r'(.+)'               # args (group 3)
        r'\)\s*;'             # close paren + semicolon
        r'(.*)$',             # trailing (group 4, typically empty or comment)
        result
    )

    if m:
        prefix = m.group(1)
        method = m.group(2)
        args = m.group(3)
        trailing = m.group(4).strip()

        # Only convert if method is not a skip method
        if not is_skip_method(method):
            # Check if prefix ends with something that suggests this is a real method call
            # e.g., _impl->_dds->Write(  or  CreateDDSPublisher(
            # We want to convert: _impl->_event.throttle()  but NOT  _impl->_dds->Init(...)

            # Skip if the method has multiple arguments (commas not inside nested parens/strings)
            # Count commas at top level (not inside parens or strings)
            depth = 0
            in_string = False
            string_char = None
            comma_count = 0
            for ch in args:
                if in_string:
                    if ch == string_char:
                        in_string = False
                    continue
                if ch in ('"', "'"):
                    in_string = True
                    string_char = ch
                elif ch == '(':
                    depth += 1
                elif ch == ')':
                    depth -= 1
                elif ch == ',' and depth == 0:
                    comma_count += 1

            if comma_count == 0:
                # Single argument -- this is likely a setter
                # Remove std::move() wrapper if present
                inner_args = args.strip()
                move_match = re.match(r'^std::move\((.+)\)$', inner_args)
                if move_match:
                    inner_args = move_match.group(1).strip()

                # Reconstruct the line
                comment = ""
                if trailing and trailing.startswith("//"):
                    comment = " " + trailing

                result = f"{prefix}.{method} = {inner_args};{comment}\n"
                return result

    return result


def convert_chained_getter_setter(line: str) -> str:
    """D. Convert chained getter().setter(val) to getter.setter = val.

    Examples:
      obj.position().x(val);     -> obj.position.x = val;
      pose.orientation().w(val); -> pose.orientation.w = val;
    """
    # Pattern: prefix.getter().setter(arg);
    # The getter() is an empty-paren call, the setter has a single arg

    # Handle chains of arbitrary depth: a.b().c().d(val);
    # Strategy: repeatedly replace .name() with .name when followed by .

    # First handle the setter at the end of a chain
    # e.g., pose.position().x(val);
    # After removing getter parens: pose.position.x(val);
    # Then convert_setter_to_assignment handles the rest.

    result = line

    # Remove empty () from getter calls that are followed by .
    # Pattern: .identifier() followed by .
    # But NOT .Init() or other method calls
    # We use a loop to handle chains like .a().b().c(val);

    def _remove_getter_parens(text):
        """Remove () from .name() when followed by . (getter in chain)."""
        prev = None
        while prev != text:
            prev = text
            # Match .identifier() followed by .
            text = re.sub(
                r'\.(\w+)\(\)(?=\.)',
                lambda m: f'.{m.group(1)}' if not is_skip_method(m.group(1)) else m.group(0),
                text
            )
        return text

    result = _remove_getter_parens(result)
    return result


def convert_getter_rvalue(line: str) -> str:
    """D (continued). Convert .field() to .field when used as rvalue.

    Examples:
      _impl->_image.width() * sizeof(uint8_t)  ->  _impl->_image.width * sizeof(uint8_t)
      _impl->_event.throttle()                  ->  _impl->_event.throttle
    """
    # Match .identifier() NOT followed by . and NOT a skip method
    # This handles rvalue access like: _impl->_event.throttle()
    # But NOT: _impl->_dds->Init()  (Init is a skip method)

    def _replace_getter(m):
        method = m.group(1)
        if is_skip_method(method):
            return m.group(0)  # Don't replace
        return f'.{method}'

    # .identifier() at end of expression (not followed by ( or .)
    # Needs to be careful: .throttle() could be end of line or in middle of expression
    result = re.sub(r'\.(\w+)\(\)(?![.(])', _replace_getter, line)
    return result


def mark_manual_fix(line: str, manual_fixes: list) -> str:
    """F. Mark patterns needing manual fix."""
    result = line
    needs_fix = False

    # CameraInfo constructor calls: sensor_msgs_msg_CameraInfo(height, width, fov)
    if re.search(r'sensor_msgs_msg_CameraInfo\s*\(\s*\w+\s*,', result):
        if "// MANUAL_FIX" not in result:
            result = result.rstrip('\n') + "  // MANUAL_FIX: CameraInfo constructor\n"
            needs_fix = True

    # Vector/sequence initializer lists: .fields({d1, d2, ...})
    # This is a setter with an initializer list argument
    if re.search(r'\.\w+\s*=\s*\{[^}]+\}\s*;', result) or re.search(r'\.\w+\(\s*\{[^}]*\}', result):
        if "// MANUAL_FIX" not in result:
            result = result.rstrip('\n') + "  // MANUAL_FIX: sequence initializer list\n"
            needs_fix = True

    # .data(std::move(...)) patterns for sequence backing store
    # After conversion this becomes .data = <expr>;
    # But we specifically look for the original pattern or the data field assignment
    # that was from a std::move of a vector
    if re.search(r'\.data\s*=\s*\w+\s*;', result) and 'data' in result:
        # Check if this looks like it was a vector move (the variable name is typically
        # vector_data, data, im_data, data_ex_raw etc.)
        var_match = re.search(r'\.data\s*=\s*(\w+)\s*;', result)
        if var_match:
            var_name = var_match.group(1)
            # Skip simple scalar data assignments
            if var_name not in ('data', 'msg', 'seconds', 'nanoseconds', '0', '1', 'false', 'true'):
                if "vector" in var_name.lower() or "raw" in var_name.lower() or var_name == "data":
                    pass  # will be caught below
        if "// MANUAL_FIX" not in result:
            result = result.rstrip('\n') + "  // MANUAL_FIX: sequence data assignment\n"
            needs_fix = True

    if needs_fix:
        manual_fixes.append(result.strip())

    return result


def convert_fields_initializer_list(line: str, manual_fixes: list) -> str:
    """Handle .fields({...}) which is a sequence setter with initializer list.
    Mark it for manual fix."""
    # This pattern: .fields({descriptor1, descriptor2, ...});
    if re.search(r'\.\w+\(\s*\{', line):
        method_match = re.search(r'\.(\w+)\(\s*\{', line)
        if method_match and not is_skip_method(method_match.group(1)):
            if "// MANUAL_FIX" not in line:
                line = line.rstrip('\n') + "  // MANUAL_FIX: sequence initializer list\n"
                manual_fixes.append(line.strip())
    return line


def process_file(input_path: Path, output_path: Path) -> str:
    """Process a single file. Returns status: 'OK', 'MANUAL_FIX', or 'SKIP'."""
    filename = input_path.name

    if filename in SKIP_FILES:
        return "SKIP"

    with open(input_path, "r") as f:
        lines = f.readlines()

    manual_fixes = []
    output_lines = []

    # Track multi-line .fields({ ... }) blocks
    in_multiline_fields = False
    multiline_fields_lines = []

    for i, line in enumerate(lines):
        # B. Remove PubSubTypes.h includes
        if remove_pubsubtypes_include(line):
            continue

        # Handle multi-line .fields({ blocks
        if in_multiline_fields:
            multiline_fields_lines.append(line)
            if '});' in line or ('});' in line.replace(' ', '')):
                # End of multi-line fields block
                combined = ''.join(multiline_fields_lines)
                combined = apply_type_mapping(combined)
                combined = convert_pointfield_constants(combined)
                if "// MANUAL_FIX" not in combined:
                    # Add MANUAL_FIX to the last line
                    combined_lines = combined.split('\n')
                    # Find the line with });
                    for idx, cl in enumerate(combined_lines):
                        if '});' in cl:
                            combined_lines[idx] = cl.rstrip('\n') + "  // MANUAL_FIX: sequence initializer list"
                            manual_fixes.append("multi-line .fields({...})")
                            break
                    combined = '\n'.join(combined_lines)
                output_lines.append(combined)
                in_multiline_fields = False
                multiline_fields_lines = []
                continue
            continue

        # Check for start of multi-line .fields({ block
        if re.search(r'\.\w+\(\s*\{\s*$', line.rstrip()):
            method_match = re.search(r'\.(\w+)\(\s*\{', line)
            if method_match and not is_skip_method(method_match.group(1)):
                in_multiline_fields = True
                processed_line = apply_type_mapping(line)
                processed_line = convert_pointfield_constants(processed_line)
                multiline_fields_lines = [processed_line]
                continue

        # A. Type name mapping
        line = apply_type_mapping(line)

        # E. PointField constants
        line = convert_pointfield_constants(line)

        # F. Check for initializer list patterns on single line: .field({...});
        if re.search(r'\.\w+\(\s*\{[^}]+\}\s*\)\s*;', line):
            method_match = re.search(r'\.(\w+)\(\s*\{', line)
            if method_match and not is_skip_method(method_match.group(1)):
                # Apply type mapping already done, mark for manual fix
                if "// MANUAL_FIX" not in line:
                    line = line.rstrip('\n') + "  // MANUAL_FIX: sequence initializer list\n"
                    manual_fixes.append(line.strip())
                output_lines.append(line)
                continue

        # F. CameraInfo constructor
        if re.search(r'sensor_msgs_msg_CameraInfo\s*\(\s*\w+\s*,', line):
            if "// MANUAL_FIX" not in line:
                line = line.rstrip('\n') + "  // MANUAL_FIX: CameraInfo constructor\n"
                manual_fixes.append(line.strip())

        # D. Convert chained getter().setter(val) patterns
        line = convert_chained_getter_setter(line)

        # D. Convert .field() rvalue access
        line = convert_getter_rvalue(line)

        # C. Convert setter calls to field assignment
        line = convert_setter_to_assignment(line)

        # F. Check for .data = pattern that was from std::move of vector data
        # After setter conversion, .data(std::move(x)) becomes .data = x;
        if re.search(r'\.data\s*=\s*\w+\s*;', line):
            # Only flag if it looks like sequence data (not scalar .data = data for Float32)
            # Heuristic: if the variable name suggests a vector
            var_match = re.search(r'\.data\s*=\s*(\w+)\s*;', line)
            if var_match:
                var_name = var_match.group(1)
                # Flag vector/array data assignments (not simple scalars)
                if any(kw in var_name.lower() for kw in ('vector', 'raw', 'im_', 'move')):
                    if "// MANUAL_FIX" not in line:
                        line = line.rstrip('\n') + "  // MANUAL_FIX: sequence data assignment\n"
                        manual_fixes.append(line.strip())

        output_lines.append(line)

    # Ensure output directory exists
    output_path.parent.mkdir(parents=True, exist_ok=True)

    # Add header comment to all generated files
    output_content = ''.join(output_lines)
    header = (
        "// *** AUTO-GENERATED by tools/generate_cyclonedds_publishers.py ***\n"
        "// Do not edit this file directly. Modify the script or the FastDDS\n"
        "// source and regenerate.\n"
    )
    if manual_fixes:
        header += (
            "//\n"
            "// MANUAL FIXES REQUIRED before this file will compile:\n"
        )
        seen = set()
        for fix in manual_fixes:
            desc = fix.split("// MANUAL_FIX:")[-1].strip() if "// MANUAL_FIX:" in fix else fix
            if desc not in seen:
                header += f"//   - {desc}\n"
                seen.add(desc)
        header += (
            "// Search for '// MANUAL_FIX' in this file to find each location.\n"
        )
    header += "\n"
    output_content = header + output_content

    with open(output_path, "w") as f:
        f.write(output_content)

    if manual_fixes:
        return "MANUAL_FIX"
    return "OK"


def main():
    print("=" * 72)
    print("CycloneDDS Publisher/Subscriber Generator")
    print("=" * 72)
    print()

    results = {"OK": [], "MANUAL_FIX": [], "SKIP": []}

    for subdir, fastdds_dir in FASTDDS_DIRS:
        if not fastdds_dir.exists():
            print(f"WARNING: Source directory not found: {fastdds_dir}")
            continue

        cyclone_dir = CYCLONE_DIRS[subdir]

        # Process all .cpp and .hpp files
        for ext in ("*.cpp", "*.hpp"):
            for src_file in sorted(fastdds_dir.glob(ext)):
                filename = src_file.name
                dst_file = cyclone_dir / filename

                status = process_file(src_file, dst_file)
                results[status].append((subdir, filename))

                # Status indicator
                if status == "SKIP":
                    print(f"  SKIP        {subdir}/{filename}")
                elif status == "MANUAL_FIX":
                    print(f"  MANUAL_FIX  {subdir}/{filename}")
                else:
                    print(f"  OK          {subdir}/{filename}")

    # Summary
    print()
    print("=" * 72)
    print("Summary")
    print("=" * 72)
    print(f"  OK:         {len(results['OK'])} files")
    print(f"  MANUAL_FIX: {len(results['MANUAL_FIX'])} files")
    print(f"  SKIP:       {len(results['SKIP'])} files")

    if results["MANUAL_FIX"]:
        print()
        print("Files needing manual attention:")
        for subdir, fn in results["MANUAL_FIX"]:
            cyclone_path = CYCLONE_DIRS[subdir] / fn
            print(f"  {cyclone_path}")

    if results["SKIP"]:
        print()
        print("Skipped files (need full manual creation):")
        for subdir, fn in results["SKIP"]:
            print(f"  {subdir}/{fn}")

    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
