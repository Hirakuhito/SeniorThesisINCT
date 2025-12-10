import trimesh


def main():
    obj_path = "./test_data.obj"

    mesh = trimesh.load_mesh(obj_path)

    mesh.show()


if __name__ == "__main__":
    main()