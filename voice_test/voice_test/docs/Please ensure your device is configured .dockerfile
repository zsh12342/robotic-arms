Please ensure your device is configured per the CUDA Tegra Setup Documentation.
wget https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2204/arm64/cuda-ubuntu2204.pin
sudo mv cuda-ubuntu2204.pin /etc/apt/preferences.d/cuda-repository-pin-600
wget https://developer.download.nvidia.com/compute/cuda/12.4.1/local_installers/cuda-tegra-repo-ubuntu2204-12-4-local_12.4.1-1_arm64.deb
sudo dpkg -i cuda-tegra-repo-ubuntu2204-12-4-local_12.4.1-1_arm64.deb
sudo cp /var/cuda-tegra-repo-ubuntu2204-12-4-local/cuda-*-keyring.gpg /usr/share/keyrings/
sudo apt-get update
sudo apt-get -y install cuda-toolkit-12-4 cuda-compat-12-4