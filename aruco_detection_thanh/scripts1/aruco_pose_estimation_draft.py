import argparse
DEFAULT_MARKER_SIZE = 26.7
def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--marker-size", type=float, default=DEFAULT_MARKER_SIZE,
                        help=f"Chieu dai canh den ngoai cung cua marker [cm] "
                             f"(mac dinh {DEFAULT_MARKER_SIZE})")



    
    parser.add_argument("--cam", type=int, default=0, help="Index camera")

    parser.add_argument("--id", type=int, default=None,
                        help="Chi hien thi marker co id nay (mac dinh: tat ca)")
    


    parser.add_argument("--side", default="left", choices=["left", "right"],
                        help="Dung intrinsic cua camera trai hay phai")
    args = parser.parse_args()


    # print(f"Marker size: {args.marker_size} cm", end="")

    print("marker-size: ", args.marker_size)
    print("cam: ", args.cam)

if __name__ == "__main__":
    main()