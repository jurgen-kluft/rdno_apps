package rdno_bedpresence

import (
	denv "github.com/jurgen-kluft/ccode/denv"
	rdno_core "github.com/jurgen-kluft/rdno_core/package"
	rdno_wifi "github.com/jurgen-kluft/rdno_wifi/package"
)

const (
	repo_path = "github.com\\jurgen-kluft"
	repo_name = "rdno_bedpresence"
)

func GetPackage() *denv.Package {
	name := repo_name

	corepkg := rdno_core.GetPackage()
	wifipkg := rdno_wifi.GetPackage()

	mainpkg := denv.NewPackage(repo_path, repo_name)
	mainpkg.AddPackage(corepkg)
	mainpkg.AddPackage(wifipkg)

	mainapp := denv.SetupCppAppProject(mainpkg, name)
	mainapp.AddDependencies(corepkg.GetMainLib()...)
	mainapp.AddDependencies(wifipkg.GetMainLib()...)

	mainpkg.AddMainApp(mainapp)
	return mainpkg
}
