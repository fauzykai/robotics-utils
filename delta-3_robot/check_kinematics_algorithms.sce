// ============================================================================
// check_kinematics_algorithms.sce
// ============================================================================

p1 = [0,0,-170]';
[ret,kp] = d3rLoadKinemConfig( "MicroDELTA-240" );

// Inverse kinematics
// ----------------------------------------------------------------------------

mprintf( "\np1 = %f,%f,%f\n",p1(1),p1(2),p1(3) );
mprintf( "tcp0 --> [IK] --> joints\n");
tcp0 = p1;
[ret,joints] = d3rInverseKinem( kp,tcp0 )
mprintf( "joints = %f,%f,%f\n\n",joints(1),joints(2),joints(3));

// Direct kinematics
// ----------------------------------------------------------------------------

mprintf( "joints = %f,%f,%f\n",joints(1),joints(2),joints(3));
mprintf( "joints --> [DK] --> tcp0\n");
[ret,p2] = d3rDirectKinem( kp,joints )
mprintf( "p2 = %f,%f,%f\n",p2(1),p2(2),p2(3) );

// ============================================================================