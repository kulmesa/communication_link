#!/bin/bash

build_nbr=$1

get_commit() {
	echo $(git rev-parse HEAD)
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
	version=$1
	echo "Creating deb package..."
	build_dir=$(mktemp -d)
	mkdir "${build_dir}/DEBIAN"
	mkdir -p "${build_dir}/usr/bin/"
	cp debian/control "${build_dir}/DEBIAN/"
	cp debian/postinst "${build_dir}/DEBIAN/"
	cp debian/prerm "${build_dir}/DEBIAN/"
	cp communication_link "${build_dir}/usr/bin/"
	cp videonode "${build_dir}/usr/bin/"

	sed -i "s/VERSION/${version}/" "${build_dir}/DEBIAN/control"
	cat "${build_dir}/DEBIAN/control"

	# create changelog
	pkg_name=$(grep -oP '(?<=Package: ).*' ${build_dir}/DEBIAN/control)
	mkdir -p ${build_dir}/usr/share/doc/${pkg_name}
	echo "${pkg_name} (${version}-0focal) focal; urgency=high" > ${build_dir}/usr/share/doc/${pkg_name}/changelog.Debian
	echo >> ${build_dir}/usr/share/doc/${pkg_name}/changelog.Debian
	echo "  * commit: $(get_commit)" >> ${build_dir}/usr/share/doc/${pkg_name}/changelog.Debian
	echo >> ${build_dir}/usr/share/doc/${pkg_name}/changelog.Debian
	echo " -- $(grep -oP '(?<=Maintainer: ).*' ${build_dir}/DEBIAN/control)  $(date +'%a, %d %b %Y %H:%M:%S %z')" >> ${build_dir}/usr/share/doc/${pkg_name}/changelog.Debian
	echo >> ${build_dir}/usr/share/doc/${pkg_name}/changelog.Debian
	gzip ${build_dir}/usr/share/doc/${pkg_name}/changelog.Debian

	echo "communication-link_${version}_amd64.deb"
	rm -rf ../communication-link_*.deb
	fakeroot dpkg-deb --build "${build_dir}" "../communication-link_${version}_amd64.deb"
	rm -rf "${build_dir}"
	echo "Done"
}

version=1.0.${build_nbr}
build
make_deb $version
