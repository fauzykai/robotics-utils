// =================================================================================
// InverseKinem.sci
// =================================================================================

function [Ret, Joints] = scrInvKinem (KinemParams, TCP0)
//
// DESCRIPTION
//  Solves the Inverse Kinematics problem of a Scara-RRP robot.
// PARAMETERS
//  KinemParams [IN] : Kinematics parameters [h,l1,l2].
//  TCP0        [IN] : Pose [x,y,z]' in RCS.
//  Joints     [OUT] : Position [j1,j2,j3]' in JCS .
// RETURN
//  Ret : Success (1) or error (<0)
// 

  Ret = 1;
  Joints = zeros(3,2);
  
  // Prepare data ---
  
  h = KinemParams(1); l1 = KinemParams(2);  l2 = KinemParams(3);
  x = TCP0(1); y = TCP0(2); z = TCP0(3);
  
  // Solve Inverse Kinematics problem ---

  // Solve RR joints
  [Ret,Theta] = __scrIkRR( [l1,l2],TCP0 );

  if Ret <> 1 then
    Ret = -1;
    Joints = zeros(3,2);      
    return [Ret,Joints];
  end
  
  // Solve prismatic joint 
   J3 = h - z;
       
  // Return the two solutions
  Joints(:,1) = [Theta(:,1); J3];
  Joints(:,2) = [Theta(:,2); J3];
  
  Ret = 1;
  return [Ret,Joints];

endfunction

// -----------------------------------------------------------------------------

function [Ret, Theta] = __scrIkRR (KinemParams, TCP0)

  Ret = 1;
  Theta = zeros(2,2);
  
  // Prepare data ---
  
  l1 = KinemParams(1);  l2 = KinemParams(2);
  x = TCP0(1); y = TCP0(2);
  
  // Solve Inverse Kinematics problem ---
  
  cosTheta2 = (x*x + y*y - l1*l1 - l2*l2) / (2*l1*l2);
  
  if abs(cosTheta2) > 1.0 then
      // Solution can not be reached	
      Ret = -1;
      return [Ret,Joints];
  else

      //Right Elbow Solution
      sinTheta2(1) = sqrt( 1-cosTheta2*cosTheta2 );
      // Left Elbow Solution
      sinTheta2(2) = -sqrt( 1-cosTheta2*cosTheta2 );

      J2(1) = atan( sinTheta2(1),cosTheta2 );
      J2(2) = atan( sinTheta2(2),cosTheta2 );

      k1 = l1 + l2*cosTheta2;
      k2(1) = l2*sinTheta2(1);
      k2(2) = l2*sinTheta2(2);
  
      aux1 = atan( y,x );
      aux2(1) = atan( k2(1),k1 );
      aux2(2) = atan( k2(2),k1 );
  
      J1(1) = aux1 - aux2(1);
      if J1(1) > %pi then
          J1(1) = J1(1) - 2*%pi;
      elseif J1(1) < -%pi then
          J1(1) = J1(1) + 2*%pi;
      end
          
      J1(2) = aux1 - aux2(2);
      if J1(2) > %pi then
          J1(2) = J1(2) - 2*%pi;
      elseif J1(2) < -%pi then
          J1(2) = J1(2) + 2*%pi;
      end

  end
  
  // Return the two solutions
  Theta(:,1) = [J1(1),J2(1)]';
  Theta(:,2) = [J1(2),J2(2)]';
  
  Ret = 1;
  return [Ret,Theta];

endfunction

// =================================================================================
 