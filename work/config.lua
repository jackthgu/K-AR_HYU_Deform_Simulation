package.resourcePath="../Resource/motion/"
package.scriptPath="../scripts/"

package.path=package.path..";./?.lua" --;"..package.path
package.path=package.path..";"..package.scriptPath.."?.lua"
package.path=package.path..";"..package.scriptPath.."/RigidBodyWin/?.lua"
package.path=package.path..";../../taesooLib/MainLib/WrapperLua/?.lua"
package.path=package.path..";../src/lua/?.lua"

require('mylib')
require('module')
