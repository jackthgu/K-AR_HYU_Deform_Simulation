require("config")

package.projectPath='../Samples/QP_controller/'
package.resourcePath='../Resource/motion/'
require("module")
require("RagdollFallSimple")

-- iguana_huge doesnt work
model_files.iguana_huge=deepCopyTable(model_files.chain)
model_files.iguana_huge.file_name="../mesh/Iguana_huge.wrl"
model_files.iguana_huge.timestep=1/8000
model_files.iguana_huge.skinScale=1
model_files.iguana_huge.initialHeight=100
--model_files.iguana.GVector=vector3(0, 1000,0)

-- 
model_files.iguana=deepCopyTable(model_files.chain)
model_files.iguana.file_name="../mesh/Iguana.wrl"
model_files.iguana.timestep=1/8000
model_files.iguana.rendering_step=1/240
model_files.iguana.skinScale=100
model_files.iguana.initialHeight=0.5
model=model_files.iguana
