#!/bin/bash
mkdir -p $HOME/rpmbuild/{SOURCES,RPMS,SRPMS}
tar czf $HOME/rpmbuild/SOURCES/opa-fm.tar.gz --exclude-vcs .
rpmbuild -ba ./opafm.spec
