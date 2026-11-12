"""Match signal emission to the stock daylight and night lighting convention.

Run in the Unreal editor after rebuilding a signal material with its original
setup script. Existing state parameters, masks, textures and colors are kept.
The saved assets are used directly by both editor and cooked builds.
"""

import math
import unreal


REFERENCE_PEAK = 100000.0
# Peak input values include the authored color and intensity parameters.
EMISSION_SCALES = {
    "/T4/TrafficLightSample/VehicleTrafficLight/Lens": REFERENCE_PEAK / 30.0,
    "/T4/Lanelet2TrafficLight/M_PedestrianFigure": REFERENCE_PEAK / (30.0 * 90.0),
    "/T4/TrafficLightSample/VehicleTrafficLight/M_JPArrowLit": REFERENCE_PEAK / 20.0,
    "/T4/TrafficLightSample/FlashingBeacon/M_JPFlashingBeaconLit": REFERENCE_PEAK / 20.0,
}
SCALE_DESCRIPTION = "Physical daylight emission scale"
OUTPUT_DESCRIPTION = "Exposure-limited signal emission"
NIGHT_SCALE = 0.01
EXPOSURE_LIMIT = 1.0
WEATHER_COLLECTION = "/Game/Carla/Blueprints/Weather/Materials/WeatherMaterialParameters"


def apply_emission_scales(save=False):
    """Add stock night dimming and a chromaticity-preserving exposure ceiling.

    Let E be eye adaptation and C the night-scaled emission. The result is
    C * min(1, EXPOSURE_LIMIT / (E * max(C.r, C.g, C.b, epsilon))).
    Thus exposed emission cannot exceed the unit linear radiance ceiling,
    and all channels receive the same gain. Zero emission remains zero.
    """
    mel = unreal.MaterialEditingLibrary
    emission = unreal.MaterialProperty.MP_EMISSIVE_COLOR
    changed = []
    saved_unsaved = []
    collection = unreal.load_asset(WEATHER_COLLECTION)
    if collection is None:
        raise RuntimeError("Missing weather material collection")
    for asset_path, scale in EMISSION_SCALES.items():
        material = unreal.load_asset(asset_path)
        if not isinstance(material, unreal.Material):
            raise RuntimeError("Missing signal material: " + asset_path)
        source = mel.get_material_property_input_node(material, emission)
        if source is None:
            raise RuntimeError("Missing emission expression: " + asset_path)
        graph_changed = True
        if source.get_editor_property("desc") == OUTPUT_DESCRIPTION:
            day = unreal.find_object(material, "SignalEmissionDayScale")
            night = unreal.find_object(material, "SignalEmissionNightScale")
            limit = unreal.find_object(material, "SignalEmissionViewLimit")
            if (math.isclose(day.get_editor_property("const_b"), scale, rel_tol=0.000001)
                    and math.isclose(night.get_editor_property("default_value"), NIGHT_SCALE, rel_tol=0.000001)
                    and math.isclose(limit.get_editor_property("default_value"), EXPOSURE_LIMIT, rel_tol=0.000001)):
                if not save:
                    continue
                # An unchanged graph may still have an unsaved package.
                dirty_packages = {
                    package.get_name()
                    for package in unreal.EditorLoadingAndSavingUtils.get_dirty_content_packages()
                }
                if material.get_path_name().split(".", 1)[0] not in dirty_packages:
                    continue
                graph_changed = False
            else:
                day.set_editor_property("const_b", scale)
                night.set_editor_property("default_value", NIGHT_SCALE)
                limit.set_editor_property("default_value", EXPOSURE_LIMIT)
        else:
            node_count = 0
            def node(cls, name, **properties):
                nonlocal node_count
                expression = mel.create_material_expression(
                    material, cls, 400 + (node_count // 4) * 240, (node_count % 4) * 240)
                node_count += 1
                expression.rename(name)
                for key, value in properties.items():
                    expression.set_editor_property(key, value)
                return expression

            def connect(left, right, pin, output=""):
                inputs = [str(value) for value in mel.get_material_expression_input_names(right)]
                if pin not in inputs and len(inputs) == 1:
                    pin = inputs[0]
                if not mel.connect_material_expressions(left, output, right, pin):
                    raise RuntimeError("Cannot connect signal emission input: " + pin)

            if (isinstance(source, unreal.MaterialExpressionMultiply)
                and source.get_editor_property("desc") == SCALE_DESCRIPTION):
                day = source
                day.rename("SignalEmissionDayScale")
            else:
                output = mel.get_material_property_input_node_output_name(material, emission)
                day = node(unreal.MaterialExpressionMultiply, "SignalEmissionDayScale", desc=SCALE_DESCRIPTION)
                connect(source, day, "A", output)
            day.set_editor_property("const_b", scale)
            factor = node(unreal.MaterialExpressionCollectionParameter, "SignalEmissionNightFactor",
                          collection=collection, parameter_name="NightFactor")
            night = node(unreal.MaterialExpressionScalarParameter, "SignalEmissionNightScale",
                         parameter_name="NightScale", default_value=NIGHT_SCALE)
            blend = node(unreal.MaterialExpressionLinearInterpolate, "SignalEmissionNightBlend", const_a=1.0)
            connect(night, blend, "B")
            connect(factor, blend, "Alpha")
            raw = node(unreal.MaterialExpressionMultiply, "SignalEmissionNightEmission")
            connect(day, raw, "A")
            connect(blend, raw, "B")
            channels = []
            for channel in "rgb":
                mask = node(unreal.MaterialExpressionComponentMask, "SignalEmissionChannel" + channel.upper(),
                            r=channel == "r", g=channel == "g", b=channel == "b", a=False)
                connect(raw, mask, "Input")
                channels.append(mask)
            maximum = node(unreal.MaterialExpressionMax, "SignalEmissionMaxRG")
            connect(channels[0], maximum, "A")
            connect(channels[1], maximum, "B")
            peak = node(unreal.MaterialExpressionMax, "SignalEmissionMaxRGB")
            connect(maximum, peak, "A")
            connect(channels[2], peak, "B")
            floor = node(unreal.MaterialExpressionMax, "SignalEmissionPeakFloor", const_b=0.000001)
            connect(peak, floor, "A")
            limit = node(unreal.MaterialExpressionScalarParameter, "SignalEmissionViewLimit",
                         parameter_name="ExposureLimit", default_value=EXPOSURE_LIMIT)
            inverse = node(unreal.MaterialExpressionEyeAdaptationInverse, "SignalEmissionInverseExposure")
            connect(limit, inverse, str(mel.get_material_expression_input_names(inverse)[0]))
            ratio = node(unreal.MaterialExpressionDivide, "SignalEmissionExposureRatio")
            connect(inverse, ratio, "A")
            connect(floor, ratio, "B")
            gain = node(unreal.MaterialExpressionMin, "SignalEmissionExposureGain", const_b=1.0)
            connect(ratio, gain, "A")
            result = node(unreal.MaterialExpressionMultiply, "SignalEmissionOutput", desc=OUTPUT_DESCRIPTION)
            connect(raw, result, "A")
            connect(gain, result, "B")
            if not mel.connect_material_property(result, "", emission):
                raise RuntimeError("Cannot connect emission output: " + asset_path)
        errors = mel.recompile_material(material)
        if errors:
            raise RuntimeError("Signal material compilation failed: %s: %s" % (asset_path, errors))
        if save and not unreal.EditorAssetLibrary.save_loaded_asset(material):
            raise RuntimeError("Cannot save signal material: " + asset_path)
        if graph_changed:
            changed.append(asset_path)
        else:
            saved_unsaved.append(asset_path)
    unreal.log("[signal_emission] changed=%d materials=%s" % (len(changed), changed))
    unreal.log("[signal_emission] saved_unsaved=%d materials=%s" % (len(saved_unsaved), saved_unsaved))
    return changed


if __name__ == "__main__":
    apply_emission_scales(save=True)
