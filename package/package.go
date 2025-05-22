package rdno_bedpresence

import (
	denv "github.com/jurgen-kluft/ccode/denv"
	rdno_core "github.com/jurgen-kluft/rdno_core/package"
	rdno_wifi "github.com/jurgen-kluft/rdno_wifi/package"
)

// GetPackage returns the package object of 'rdno_bedpresence'
func GetPackage() *denv.Package {
	// Dependencies
	ucorepkg := rdno_core.GetPackage()
	uwifipkg := rdno_wifi.GetPackage()

	// The main (rdno_bedpresence) package
	mainpkg := denv.NewPackage("rdno_bedpresence")
	mainpkg.AddPackage(ucorepkg)
	mainpkg.AddPackage(uwifipkg)

	// 'rdno_bedpresence' application
	mainapp := denv.SetupCppAppProject("rdno_bedpresence", "github.com\\jurgen-kluft\\rdno_bedpresence")
	mainapp.AddDependencies(ucorepkg.GetMainLib()...)
	mainapp.AddDependencies(uwifipkg.GetMainLib()...)

	mainpkg.AddMainApp(mainapp)
	return mainpkg
}
