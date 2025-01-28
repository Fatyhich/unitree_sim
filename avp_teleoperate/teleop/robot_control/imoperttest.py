import sys
print("Python version:", sys.version)
print("Python path:", sys.path)

try:
    import pinocchio
    print("Pinocchio imported successfully")
    print("Pinocchio version:", pinocchio.__version__)
    print("Pinocchio location:", pinocchio.__file__)
except ImportError as e:
    print("Failed to import pinocchio")
    print("Error:", str(e))