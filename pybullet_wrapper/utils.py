COLOR_RED = "\033[1;31m"
COLOR_YELLOW = "\033[1;33m"
COLOR_OFF = "\033[0m"

def debug(message):
    """Print debug message with color."""
    print(f"{COLOR_YELLOW}{message}{COLOR_OFF}")

def error(message):
    """Print debug message with color, and exit."""
    print(f"{COLOR_RED}{message}{COLOR_OFF}")
    raise