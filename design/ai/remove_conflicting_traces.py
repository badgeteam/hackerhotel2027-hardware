#!/usr/bin/env python3
"""Delete the specific tracks/vias that DRC flagged as now conflicting
with pads after the 45-degree rotation snap (identified by UUID from the
DRC report, not a broad net-based guess -- these 22 items are precisely
the ones DRC says collide with a moved LED's pad)."""
from kipy import KiCad

TARGET_UUIDS = {
    "043bd0fb-f89f-4769-bdd2-80811ac01e7e",
    "bc156404-5b1a-4f6d-8e8c-0e324fd1a5fe",
    "148f95ae-7562-4444-b418-20c8675b1cb9",
    "f8c8be9c-e689-428f-a4e5-4d241cfea9fd",
    "4771e7b6-8b41-4a06-a6e8-ba882e8f9b1e",
    "50de3639-2a57-45c3-84c1-bc19cbf9eda7",
    "c74bd8a6-7dc1-4737-9f4e-7a2eb3aab6ef",
    "fe4d1543-87a8-442d-9857-714c89f9613d",
    "fcaec0d8-6131-4672-8ee1-0e80abd7b504",
    "bd44bd9b-0269-417c-9a5e-2cfee27cd7f0",
    "9f4673a5-685b-48d5-a458-60c20264cb77",
    "299491ee-82c2-4a1f-b9db-2010eda9ecd3",
    "5dd78de7-41c6-4b60-989d-a78c475bf678",
    "f4f32cbc-9675-4516-a15a-fcb0c1325cd3",
    "4f07c9d1-edef-4285-b256-e1c278435248",
    "f0fdae74-a309-4897-8d28-f271969cc690",
    "9386e071-4ec2-48e7-8fbe-84602ea9f090",
    "4a974ffa-f10c-4c78-a440-0b8f15e9e0d6",
    "7e7cbbc6-8b0e-4443-b817-7a2ae98042e9",
    "12532101-a6d5-493e-8b2f-5abec3a41391",
    "a388e104-a522-4e59-9537-2bf99e3291f7",
    "c1885dfd-81b3-48d9-a717-2bce096c9a52",
}


def main():
    kicad = KiCad()
    board = kicad.get_board()

    items = list(board.get_tracks()) + list(board.get_vias())
    to_remove = [it for it in items if it.id.value in TARGET_UUIDS]
    print(f"Found {len(to_remove)} of {len(TARGET_UUIDS)} target items")

    board.remove_items(to_remove)
    print(f"Removed {len(to_remove)} tracks/vias.")


if __name__ == "__main__":
    main()
