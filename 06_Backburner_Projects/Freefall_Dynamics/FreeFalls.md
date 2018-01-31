For the algorithm:

dt_win_(p.maxdt()/2.0),
x_(p.xlen()),
rho_(p.xlen()), 
u_curr_(p.ulen()),
xdot_(u),
rho_dot_(x_intp, J1_, p),
m_mrho_tf_( p.xlen(), 1 ),
u2Opt_( x_intp, rho_intp, alpha_, p ),
dJdlam_( x_intp, rho_intp, u2Opt_, p ),
cntrlCost_( u2Opt_, dJdlam_, p ),
J1_(x_intp, p), p_(p),
t_i(p.T()), t_f(p.T()), tf(p.T()), 
x_intp(x_vec , times, p.xlen()),
rho_intp(rho_vec , rho_times, p.xlen()),
x0noU(p.xlen()), u(p)


    /*!
      Performs an iteration of SAC control.
      \param[in,out] t0 initial time associated with state vector input x0.  
      This get updated by one sample time to t0=t0+ts after stepper completes.
      \param[in,out] x0 initial state vector to be integrated forward after one
      iteration of SAC.  The stepper integrates x0 from time t0 to time t0+ts 
      based on SAC controls.
      \param[in,out] u1 default control value to apply from t0 to t0+calc_tm.
      The stepper computes the new SAC control to appy from t0+calc_tm to 
      t0+calc_tm+ts and returns it in u1.
     */

/* Initialize final time */
tf = initial_time + time_horizon

/* Simulate initial trajectory */
Compute Adjoint with u1 between (initial_time, initial_time + time_horizon)
SimInitXRho( t0, x0, u1 );

/* Initialize Cost, timestep, & iters before applying u2 */
Compute J1

J0 = J1;
Jn = J0;

dtt_win_ = dt_win_;
iteration_counter = 0;

/* Set alpha based on the initial cost */
alpha_ = -10000.0;//p_.lam()*J0;

/* u2 automatically computed from x_intp - find opt time to apply */
application_time = initial_time + calc_time

u2Opt_( t_app, u_curr_ );
dJdlam_( t_app, dJdlam_curr_ );
    
tau_initial = application_time - dtt_win_;
tau_final = application_time + dtt_win_;

if ( t_f < t_i ) { t_i = t_f; }
else if ( dJdlam_curr_ < 0 ) {  // use control only if dJdlam < 0
	/* Simulate X based on applying u2* at optimal time */
	SimNewX( t0, x0, u1 );
      
	/* Update Cost after applying u2* */
	J1_.update();	Jn = J1_;

	/* Backtrack until cost is improved by desired amount or its */
	while ( ( (Jn>J0) || (std::abs(Jn-J0)<0.01*J0) ) 
		&& (its<p_.backtrack_its()) ) {
	  dtt_win_ = dtt_win_/2.0;
	  t_i = (t_app-dtt_win_); t_f = (t_app+dtt_win_);
	  if ( t_i < t0+p_.calc_tm() ) { t_i = t0+p_.calc_tm(); }  
	  if ( t_f > tf ) { t_f=tf; } if ( t_i>=t_f ) { t_i=t_f; break; }
	  else {  
	    /* Simulate X based on applying new duration */
	    SimNewX( t0, x0, u1 );
      
	    /* Update Cost after applying u2* */
	    J1_.update();      Jn = J1_;	  
	  } /* end else */
	  its++;
	} /* end backtracking while */
      
      } /* end else if */
else { Jn = J0+1; }

      if ( t_f > t0+p_.ts()+p_.calc_tm() ) { t_f = t0+p_.ts()+p_.calc_tm(); }  
      if ( t_f < t_i ) { t_i = t_f; }
      
      if ( Jn > J0 ) { // cost increased so don't apply control
      	x0 = x0noU;    // return state under default control
	t_i=t0+p_.calc_tm(); t_app=t_i; 
      	t_f=t_i+p_.ts();    // update control horizon
	u1.clear(); u1.stimes(t_i, t_f); // return default u1 over horizon
      }
      else { 
      	x_intp( t0+p_.ts(), x0 ); // return updated state
	// return new control with new horizon {t_i, t_app, t_f}
	u1=u_curr_; u1.stimes(t_i, t_f);
      }
      t0=t0+p_.ts(); // return updated time
    }

  };