from multiverse_parser import InertiaSource, LightwheelImporter, MjcfExporter
from multiverse_parser import configure_logging, logging

def main():
    level = logging.DEBUG
    configure_logging(level=level)

    input_path = "/media/giangnguyen/Storage/Ubuntu2204/Lightwheel_Kitchen/Collected_KitchenRoom/KitchenRoom.usd"
    output_path = "/media/giangnguyen/Storage/Ubuntu2204/Multiverse-Parser/Kitchen.xml"
    factory = LightwheelImporter(
        file_path=input_path,
        with_visual=True,
        with_collision=True,
        inertia_source=InertiaSource.FROM_SRC,
        black_list_names=["Kitchen_Ground", "Plane", "Kitchen_Flowers001"]
    )
    factory.import_model()
    exporter = MjcfExporter(file_path=output_path, factory=factory)
    exporter.build()
    exporter.export(keep_usd=True)

if __name__ == "__main__":
    main()
