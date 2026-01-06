from bagfile_loader import BagfileSectionExtractor

bag_path = "bags_2025-07-20/manual_driving_data_collection_2025-07-20-23-23-38.bag"


extractor = BagfileSectionExtractor(bag_path=bag_path, min_window_size=10, debug=False)
extractor.load()
extractor.create_windows()

# extractor.plot_xy_coordinates(use_windows=True)
extractor.plot_xy_coordinates_and_centers()
