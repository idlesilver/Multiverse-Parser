from multiverse_parser import InertiaSource, UsdImporter, MjcfExporter

import logging
logging.basicConfig(level=logging.INFO)

def main():
    input_path = "/home/galbot/Projects/reason_vla/experiments/usd2mjcf/dryer-5f/dryer_no_material.usd"
    output_path = "/home/galbot/Projects/reason_vla/experiments/usd2mjcf/dryer-5f/MJCF/dryer_no_material.xml"
    factory = UsdImporter(file_path=input_path,
                           fixed_base=True,
                        #    root_name="HongGanJi01", # Or robot root link
                           with_physics=True,
                           with_visual=True,
                           with_collision=True,
                           inertia_source=InertiaSource.FROM_SRC)
    factory.import_model()
    exporter = MjcfExporter(file_path=output_path,
                            factory=factory)
    exporter.build()
    exporter.export()

if __name__ == "__main__":
    main()