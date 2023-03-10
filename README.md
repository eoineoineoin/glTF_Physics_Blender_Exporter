## Experimental extension to export rigid body information in a glTF file.

This is a work in progress, used to validate the concepts used by the proposed
glTF specification for rigid body dynamics, [MSFT_Physics](https://github.com/eoineoineoin/glTF_Physics)

## Installation

1. Clone this repository.
2. Open Blender
3. Navigate to Edit -> Preferences -> Addons
4. Press "Install"
5. Select the directory which contains this README file.
6. Browse to and enable the addon "Import-Export: MSFT_Physics"

## Known limitations

Blender's "cone" shapes are not exported correctly.

Collision geometry may not have the minimal required volume for that particular shape type if vertices are offset from node.

Blender's UI hides collision filtering information for children of a "Compound Parent" body - to set the collision filtering information for such an object, the child needs to be temporarily unparented from the compound.

This extension does not yet handle importing glTF assets which use the Physics extension
