package rdno_apps

import (
	denv "github.com/jurgen-kluft/ccode/denv"
	rdno_sensors "github.com/jurgen-kluft/rdno_sensors/package"
	rdno_wifi "github.com/jurgen-kluft/rdno_wifi/package"
)

const (
	repo_path = "github.com\\jurgen-kluft"
	repo_name = "rdno_apps"
)

func GetPackage() *denv.Package {
	wifipkg := rdno_wifi.GetPackage()
	sensorspkg := rdno_sensors.GetPackage()

	mainpkg := denv.NewPackage(repo_path, repo_name)
	mainpkg.AddPackage(wifipkg)
	mainpkg.AddPackage(sensorspkg)

	// Setup the main applications
	airquality := denv.SetupCppAppProject(mainpkg, "airquality", "airquality")
	airquality.AddDependencies(wifipkg.GetMainLib())
	airquality.AddDependencies(sensorspkg.GetMainLib())
	airquality.AddSourceFiles("common", ".cpp")

	humanpresence := denv.SetupCppAppProject(mainpkg, "humanpresence", "humanpresence")
	humanpresence.AddDependencies(wifipkg.GetMainLib())
	humanpresence.AddDependencies(sensorspkg.GetMainLib())
	humanpresence.AddSourceFiles("common", ".cpp")

	magnet := denv.SetupCppAppProject(mainpkg, "magnet", "magnet")
	magnet.AddDependencies(wifipkg.GetMainLib())
	magnet.AddDependencies(sensorspkg.GetMainLib())
	magnet.AddSourceFiles("common", ".cpp")

	rd03d := denv.SetupCppAppProject(mainpkg, "rd03d", "rd03d")
	rd03d.AddDependencies(wifipkg.GetMainLib())
	rd03d.AddDependencies(sensorspkg.GetMainLib())
	rd03d.AddSourceFiles("common", ".cpp")

	mainpkg.AddMainApp(airquality)
	mainpkg.AddMainApp(humanpresence)
	mainpkg.AddMainApp(magnet)
	mainpkg.AddMainApp(rd03d)

	return mainpkg
}
