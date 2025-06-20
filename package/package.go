package rdno_apps

import (
	denv "github.com/jurgen-kluft/ccode/denv"
	rdno_core "github.com/jurgen-kluft/rdno_core/package"
	rdno_sensors "github.com/jurgen-kluft/rdno_sensors/package"
	rdno_wifi "github.com/jurgen-kluft/rdno_wifi/package"
)

const (
	repo_path = "github.com\\jurgen-kluft"
	repo_name = "rdno_apps"
)

func GetPackage() *denv.Package {
	corepkg := rdno_core.GetPackage()
	wifipkg := rdno_wifi.GetPackage()
	sensorspkg := rdno_sensors.GetPackage()

	mainpkg := denv.NewPackage(repo_path, repo_name)
	mainpkg.AddPackage(corepkg)
	mainpkg.AddPackage(wifipkg)
	mainpkg.AddPackage(sensorspkg)

	// Setup the main applications
	bedpresence := denv.SetupCppAppProject(mainpkg, "bedpresence", "bedpresence")
	bedpresence.AddDependencies(corepkg.GetMainLib()...)
	bedpresence.AddDependencies(wifipkg.GetMainLib()...)

	airquality := denv.SetupCppAppProject(mainpkg, "airquality", "airquality")
	airquality.AddDependencies(corepkg.GetMainLib()...)
	airquality.AddDependencies(wifipkg.GetMainLib()...)
	airquality.AddDependencies(sensorspkg.GetMainLib()...)

	mainpkg.AddMainApp(bedpresence)
	mainpkg.AddMainApp(airquality)
	return mainpkg
}
