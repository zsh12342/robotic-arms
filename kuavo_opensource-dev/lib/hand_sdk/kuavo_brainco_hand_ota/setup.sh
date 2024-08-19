cd libserialport/
./autogen.sh && ./configure && make && sudo make install
sudo ldconfig
cd ../ && mkdir build && cd build && cmake .. && make