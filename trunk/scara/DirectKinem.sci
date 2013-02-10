// =================================================================================
// DirectKinem.sci
// =================================================================================

function [Ret, TCP0] = scrDirKinem (KinemParams, Joints)
//
// DESCRIPTION
//  Solves the Direct Kinematics problem of a Scara-RRP robot. 
// PARAMETERS
//  KinemParams [IN] : Kinematics parameters [h,l1,l2].
//  Joints      [IN] : Position [j1,j2,j3]' in JCS.
//  TCP0       [OUT] : Pose [x,y,z]' in RCS.
// RETURN
//  Ret : Success (1) or error (<0)
//

  Ret = 1;
  TCP0 = zeros(3,1);
  
  // Prepare data ---
  
  h = KinemParams(1); l1 = KinemParams(2);  l2 = KinemParams(3);
  J = Joints;

  // Solve Direct Kinematics problem ---
  
  // Solve RR joints
  [Ret,TCP0] = __scrDkRR( [l1,l2],J(1:2) );
 
  if Ret <> 1 then
     Ret = -1;
     Joints = zeros(2,3);      
     return [Ret,Joints];
   end
  
  // Solve prismatic joint 
  z = h - J(3);
  
  // Return the solutions  
  
  TCP0(3) = z;
  
  Ret = 1;
  return [Ret,TCP0];

endfunction

// -----------------------------------------------------------------------------

function [Ret, TCP0] = __scrDkRR (KinemParams, Joints)

  Ret = 1;
  TCP0 = zeros(3,1);
  
  // Prepare data ---
  
  L = KinemParams;
  J = Joints;

  // Solve Direct Kinematics problem ---
  
  TCP0(1) = L(1)*cos(J(1)) + L(2)*cos(J(1)+J(2));  
  TCP0(2) = L(1)*sin(J(1)) + L(2)*sin(J(1)+J(2));
  TCP0(3) = 0.0;

endfunction

// =============================================================================

