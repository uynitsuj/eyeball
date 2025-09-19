import logging
import logging.config
import os


def setup_logging(config_path: str | None = None) -> None:
    """Setup logging for the application."""
    # Load logging config from file if it exists
    if config_path and os.path.exists(config_path):
        logging.config.fileConfig(config_path)
    else:
        logging.basicConfig(
            level=logging.INFO,
            format="%(asctime)s.%(msecs)03dZ - %(levelname)s - %(name)s - %(message)s",
            datefmt="%Y-%m-%dT%H:%M:%S",
            force=True,
        )