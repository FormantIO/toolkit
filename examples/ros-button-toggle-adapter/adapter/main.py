
from adapter import Adapter

def main():
    try:
        adapter = Adapter()
        adapter.start()
    except KeyboardInterrupt:
        adapter.shutdown()

if __name__ == "__main__":
    main()