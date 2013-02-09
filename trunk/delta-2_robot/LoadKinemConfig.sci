// =================================================================================
// LoadKinemConfig.sci
// =================================================================================

function [Ret, KinemParams] = d2rLoadKinemConfig (Model)
//
// DESCRIPTION
//  Load configuration (rf,lf,le,re) parameters of a delta-2 robot
//  The model type is specificated using 'Model' input which is a string. If the model 
//  does not exist the function returns an error.
//  
// PARAMETERS
//  Model        [IN] : Model name (string).
//  KinemParams [OUT] : Kinematics parameters (rf,lf,le,re) [1x4].
// RETURN
//  Ret : Success (1) or error (<0)
//

  Ret = 1;
  
  select Model   
  
  // UG-D2 from CODIAN
  // http://www.codian-robotics.com/
  case "UG-D2"
    rf = 150.0;
    lf = 401.5;
    le = 850.0;
    re  = 0.0;
    
  // Unknown model      
  else
    errMsg = sprintf( "Unknown robot model: %s",Model );
    
    Ret = -1;
    error( errMsg ); // By defaut error stops execution and resume to prompt level.
  end
  
  KinemParams(1) = rf;
  KinemParams(2) = lf;
  KinemParams(3) = le;
  KinemParams(4) = re;

  return [Ret,KinemParams];
  
endfunction 

// =================================================================================
