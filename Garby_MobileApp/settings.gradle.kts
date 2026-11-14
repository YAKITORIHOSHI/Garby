pluginManagement {
    repositories {
        google {
            content {
                includeGroupByRegex("com\\.android.*")
                includeGroupByRegex("com\\.google.*")
                includeGroupByRegex("androidx.*")
            }
        }
        mavenCentral()
        gradlePluginPortal()
    }
}

dependencyResolutionManagement {
    repositoriesMode.set(RepositoriesMode.FAIL_ON_PROJECT_REPOS)
    repositories {
        google()
        mavenCentral()
    }
}

rootProject.name = "Garby"
include(":app")

// A checkout on a network share (such as the Q: drive) can fail Gradle's build-directory I/O
// with "NoSuchFileException" or "An unexpected network error occurred". A machine can opt in to
// keeping build outputs on a local disk by setting an absolute `garby.buildRoot` in its own
// local.properties (never committed). Without it, outputs stay in each project's build/.
val garbyBuildRoot: File? = rootDir.resolve("local.properties")
    .takeIf { it.isFile }
    ?.let { file -> java.util.Properties().apply { file.inputStream().use { load(it) } } }
    ?.getProperty("garby.buildRoot")
    ?.trim()
    ?.takeIf { it.isNotEmpty() }
    ?.let(::File)
    ?.takeIf { it.isAbsolute }

if (garbyBuildRoot != null) {
    gradle.beforeProject {
        val name = if (path == ":") "root" else path.removePrefix(":").replace(':', '_')
        layout.buildDirectory.set(garbyBuildRoot.resolve(name))
    }
}
