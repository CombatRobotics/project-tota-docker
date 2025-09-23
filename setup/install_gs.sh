sudo apt update

# Install gstreamer
sudo apt-get install -y libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-bad1.0-dev gstreamer1.0-plugins-base gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly gstreamer1.0-libav gstreamer1.0-tools gstreamer1.0-x gstreamer1.0-alsa gstreamer1.0-gl gstreamer1.0-gtk3 gstreamer1.0-qt5 gstreamer1.0-pulseaudio

sudo apt install -y dkms git python3-all-dev net-tools virtualenv fakeroot debhelper python3-twisted
 libpcap-dev python3-pyroute2 python3-future python3-configparser python3-all libsodium-dev

mkdir -p ~/.tota/tota_gs

cd ~/.tota/tota_gs

git clone https://github.com/jedisct1/libsodium --branch stable
cd libsodium
./configure
make && make check
sudo make install

cd ..

git clone -b v5.2.20 https://github.com/svpcom/rtl8812au.git
cd rtl8812au/
sudo ./dkms-install.sh
