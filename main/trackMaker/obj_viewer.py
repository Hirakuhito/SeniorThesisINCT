import matplotlib.pyplot as plt
import trimesh


def main():
    file_path = "../trackData/test_circuit.obj"

    try:
        mesh = trimesh.load(file_path)
    
    except IOError as e:
        print(f"Can not open : {file_path}")
        exit()

    print("Mesh loaded SuccessFully.")
    print(f"Number of vartices : {len(mesh.vertices)}")
    print(f"Number of faces : {len(mesh.faces)}")

if __name__ == "__main__":
    main()