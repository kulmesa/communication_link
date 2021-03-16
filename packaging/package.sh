#!/bin/bash

get_version() {
	version=1.0."${BUILD_NUMBER}"~$(git describe --always --tags --dirty --match "[0-9]*.[0-9]*.[0-9]*")
	echo "${version}"
}

build() {
	cd ..
	go mod download || exit
	go build || exit
	cp -f communication_link packaging/ && go clean

	cd videonode || exit
	go mod download || exit
	go build || exit
	cp -f videonode ../packaging/ && go clean
	cd ../packaging/ || exit
}

make_deb() {
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir "${build_dir}/DEBIAN"
	mkdir -p "${build_dir}/usr/bin/"
	cp debian/control "${build_dir}/DEBIAN/"
	cp debian/postinst "${build_dir}/DEBIAN/"
	cp debian/prerm "${build_dir}/DEBIAN/"
	cp communication_link "${build_dir}/usr/bin/"
	cp videonode "${build_dir}/usr/bin/"

	get_version
	sed -i "s/VERSION/${version}/" "${build_dir}/DEBIAN/control"
	cat "${build_dir}/DEBIAN/control"
	echo "communication-link_${version}_amd64.deb"
	rm -rf ../communication-link_*.deb
	fakeroot dpkg-deb --build "${build_dir}" "../communication-link_${version}_amd64.deb"
	rm -rf "${build_dir}"
	echo "Done"
}

BUILD_NUMBER=${1:-0}

version=$(get_version)
build
make_deb
