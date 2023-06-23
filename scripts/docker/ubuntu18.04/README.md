# Build image and run container

This Dockerfile that builds an image containing opensim-core in Ubuntu 18.04.

To build the image execute the following command:

    docker build --progress=plain -t opensim-ubuntu:ubuntu-18.04 .
	
To run the resulting container, execute the following command:

	docker run -i -v opensim_volume:/root/artifacts/ -t opensim-ubuntu:ubuntu-18.04
	
This command creates a volume with access to the folder `/root/artifacts`, containing the built packages.

# Download artifacts

## From Docker Desktop

1. Open `Docker Desktop`.
2. Click on `Volumes` in the left bar.
3. Click on `opensim_volume`.
4. Select the `Data` tab at the top.
5. Right click on each file and `Save as...`.

## From console

1. If you are inside of the container, type `exit`.
2. Insert the following command to download the artifacts:

	docker cp <container-name>:/root/artifacts/opensim-core.7z C:\Users\<user-name>\Downloads