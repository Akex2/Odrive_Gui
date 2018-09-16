import sys
#!/usr/bin/env python3
"""
ODrive Gui
"""

from PyQt5.QtCore import pyqtSlot
from PyQt5.QtWidgets import QApplication,QDialog, QTabWidget
from PyQt5.uic import loadUi
from PyQt5.QtCore import *
from PyQt5.QtGui import *

import odrive

from odrive.enums import *

import time

import math



# Find a connected ODrive (this will block until you connect one)
print("shearch odrive")
my_drive = odrive.find_any()  #//////////////////////////////////////////////////////////////////
print("odrive find")





class odriveUI(QTabWidget):
	def __init__(self):
		super(odriveUI,self).__init__()
		loadUi('odriveUI.ui',self)
		self.setWindowTitle('Odrive Configurator PyQt5 Gui')
		self.pushButton.clicked.connect(self.on_pushButton_clicked)
		#Connect push buton get axis parameter to def
		self.pbgetaxisparam.clicked.connect(self.getaxis_clicked)
		#Connect push buton get motor parameter to def
		self.pbgetmotorparam.clicked.connect(self.getmotor_clicked)
		#Connect push buton get controller parameter to def
		self.pbgetcontrolerparam.clicked.connect(self.getcontroller_clicked)
		#Connect push buton get encoder parameter to def
		self.pbgetencoderparam.clicked.connect(self.getencoder_clicked)
		#Connect push buton get odrv0 parameter to def
		self.pbgetodrv0.clicked.connect(self.getodrv0_clicked)
		#Connect push buton get sensorless estimator parameter to def
		self.pbgetsensorless_estimator.clicked.connect(self.getsensorless_estimator_clicked)

		#Connect set buton of odrv0 to def
		self.pbsetbrake_resistance.clicked.connect(self.brake_resistance_clicked)
		#Connect set buton of odrv0 to def (config)
		self.pbsetenable_uart.clicked.connect(self.enable_uart_clicked)
		self.pbsetenable_i2c_instead_of_can.clicked.connect(self.enable_i2c_instead_of_can_clicked)
		self.pbsetenable_ascii_protocol_on_usb.clicked.connect(self.enable_ascii_protocol_on_usb_clicked)
		self.pbsetdc_bus_undervoltage_trip_level.clicked.connect(self.dc_bus_undervoltage_trip_level_clicked)
		self.pbsetdc_bus_overvoltage_trip_level.clicked.connect(self.dc_bus_overvoltage_trip_level_clicked)
		
		#Connect set buton of Axis to def
		self.pbsetcurrentstate.clicked.connect(self.requested_state_clicked)		
		self.pbsetaxiserror0.clicked.connect(self.axiserror0_clicked)
		#Connect set buton of Axis to def (config)
		#self.pbsetenablemotorpin.clicked.connect(self.setenablemotorpin_clicked)
		self.pbsetenablestepdir.clicked.connect(self.setenablestepdir_clicked)
		self.pbsetstartup_motor_calibration.clicked.connect(self.startup_motor_calibration_clicked)
		self.pbsetstartup_encoder_index_search.clicked.connect(self.startup_encoder_index_search_clicked)
		self.pbsetstartup_encoder_offset_calibration.clicked.connect(self.startup_encoder_offset_calibration_clicked)
		self.pbsetstartup_closed_loop_control.clicked.connect(self.startup_closed_loop_control_clicked)
		self.pbsetstartup_sensorless_control.clicked.connect(self.startup_sensorless_control_clicked)		
		self.pbsetcounts_per_step.clicked.connect(self.counts_per_step_clicked)
		self.pbsetramp_up_time.clicked.connect(self.ramp_up_time_clicked)
		self.pbsetramp_up_distance.clicked.connect(self.ramp_up_distance_clicked)
		self.pbsetspin_up_current.clicked.connect(self.spin_up_current_clicked)
		self.pbsetspin_up_acceleration.clicked.connect(self.spin_up_acceleration_clicked)
		self.pbsetspin_up_target_vel.clicked.connect(self.spin_up_target_vel_clicked)
		

		
		#Connect set buton of Motor to def
		self.pbsetmotorerror0.clicked.connect(self.motorerror0_clicked)
		#Connect set buton of Motor to def (config)
		self.pbsetpre_calibrated_2.clicked.connect(self.pre_calibrated_2_clicked)
		self.pbsetpole_pairs.clicked.connect(self.pole_pairs_clicked)
		self.pbsetcalibration_current.clicked.connect(self.calibration_current_clicked)
		self.pbsetresistance_calib_max_voltage.clicked.connect(self.resistance_calib_max_voltage_clicked)
		self.pbsetphase_inductance.clicked.connect(self.phase_inductance_clicked)
		self.pbsetphase_resistance.clicked.connect(self.phase_resistance_clicked)
		self.pbsetdirection.clicked.connect(self.direction_clicked)
		self.pbsetmotor_type.clicked.connect(self.motor_type_clicked)
		self.pbsetcurrent_lim.clicked.connect(self.current_lim_clicked)
		self.pbsetrequested_current_range.clicked.connect(self.requested_current_range_clicked)
		self.pbsetcurrent_control_bandwidth.clicked.connect(self.current_control_bandwidth_clicked)


		
		#Connect set buton of Controller to def (config)
		self.pbsetcontrol_mode.clicked.connect(self.control_mode_clicked)
		self.pbsetpos_gain.clicked.connect(self.pos_gain_clicked)
		self.pbsetvel_gain.clicked.connect(self.vel_gain_clicked)
		self.pbsetvel_integrator_gain.clicked.connect(self.vel_integrator_gain_clicked)
		self.pbsetvel_limit.clicked.connect(self.vel_limit_clicked)
		self.pbsetenable_gain_scheduling.clicked.connect(self.enable_gain_scheduling_clicked)
		self.pbsetgain_scheduling_width.clicked.connect(self.gain_scheduling_width_clicked)

		#Connect set buton of Encoder to def
		self.pbsetencodererror0.clicked.connect(self.encodererror0_clicked)
		#Connect set buton of Encoder to def (config)
		self.pbsetencoder_mode.clicked.connect(self.mode_clicked)
		self.pbsetuse_index.clicked.connect(self.use_index_clicked)
		self.pbsetpre_calibrated.clicked.connect(self.pre_calibrated_clicked)
		self.pbsetidx_search_speed.clicked.connect(self.idx_search_speed_clicked)
		self.pbsetcpr.clicked.connect(self.cpr_clicked)
		self.pbsetoffset.clicked.connect(self.offset_clicked)
		self.pbsetoffset_float.clicked.connect(self.offset_float_clicked)
		self.pbsetbandwidth.clicked.connect(self.bandwidth_clicked)
		self.pbsetcalib_range.clicked.connect(self.calib_range_clicked)

		#Connect set buton of Sensorless estimator to def
		self.pbsetsensorless_estimatorerror.clicked.connect(self.sensorless_estimator_error_clicked)
		self.pbsetobserver_gain.clicked.connect(self.observer_gain_clicked)
		self.pbsetpll_bandwidth.clicked.connect(self.pll_bandwidth_clicked)
		self.pbsetpm_flux_linkage.clicked.connect(self.pm_flux_linkage_clicked)

		#Ajout ComboBox//////////////////////////////////////////////////////////////////////////////////////
		#Ajout ComboBox Odrv0 tab
		self.cbenable_uart.addItem("False")
		self.cbenable_uart.addItem("True")
		self.cbenable_i2c_instead_of_can.addItem("False")
		self.cbenable_i2c_instead_of_can.addItem("True")
		self.cbenable_ascii_protocol_on_usb.addItem("False")
		self.cbenable_ascii_protocol_on_usb.addItem("True")


		#Ajout ComboBox motor tab
		self.cbrequested_state.addItem("")
		self.cbrequested_state.addItem("AXIS_STATE_IDLE")
		self.cbrequested_state.addItem("AXIS_STATE_STARTUP_SEQUENCE")
		self.cbrequested_state.addItem("AXIS_STATE_FULL_CALIBRATION_SEQUENCE")
		self.cbrequested_state.addItem("AXIS_STATE_MOTOR_CALIBRATION")
		self.cbrequested_state.addItem("AXIS_STATE_SENSORLESS_CONTROL")
		self.cbrequested_state.addItem("AXIS_STATE_ENCODER_INDEX_SEARCH")
		self.cbrequested_state.addItem("AXIS_STATE_ENCODER_OFFSET_CALIBRATION")
		self.cbrequested_state.addItem("AXIS_STATE_CLOSED_LOOP_CONTROL")

		self.cbenablemotorpin.addItem("False")
		self.cbenablemotorpin.addItem("True")
		self.cbenablestepdir.addItem("False")
		self.cbenablestepdir.addItem("True")

		self.cbstartup_motor_calibration.addItem("False")
		self.cbstartup_motor_calibration.addItem("True")
		self.cbstartup_encoder_index_search.addItem("False")
		self.cbstartup_encoder_index_search.addItem("True")
		self.cbstartup_closed_loop_control.addItem("False")
		self.cbstartup_closed_loop_control.addItem("True")
		self.cbstartup_sensorless_control.addItem("False")
		self.cbstartup_sensorless_control.addItem("True")
		self.cbstartup_encoder_offset_calibration.addItem("False")
		self.cbstartup_encoder_offset_calibration.addItem("True")

		self.cbpre_calibrated_2.addItem("False")
		self.cbpre_calibrated_2.addItem("True")
		self.cbdirection.addItem("-1")
		self.cbdirection.addItem("1")
		self.cbmotor_type.addItem("MOTOR_TYPE_HIGH_CURRENT")
		self.cbmotor_type.addItem("MOTOR_TYPE_GIMBAL")

		#Ajout ComboBox Controller tab
		self.cbcontrol_mode.addItem("CTRL_MODE_VOLTAGE_CONTROL")
		self.cbcontrol_mode.addItem("CTRL_MODE_CURRENT_CONTROL")
		self.cbcontrol_mode.addItem("CTRL_MODE_VELOCITY_CONTROL")
		self.cbcontrol_mode.addItem("CTRL_MODE_POSITION_CONTROL")

		self.cbenable_gain_scheduling.addItem("False")
		self.cbenable_gain_scheduling.addItem("True")

		#Ajout ComboBox Encoder tab
		self.cbencoder_mode.addItem("MODE_INCREMENTAL")
		self.cbencoder_mode.addItem("MODE_HALL")
		self.cbuse_index.addItem("False")
		self.cbuse_index.addItem("True")
		self.cbpre_calibrated.addItem("False")
		self.cbpre_calibrated.addItem("True")
		self.cbpre_calibrated.addItem("False")
		self.cbpre_calibrated.addItem("True")



	@pyqtSlot()
	def on_pushButton_clicked(self):
		print("save...")
		my_drive.save_configuration()

	#Get odrv0 param fuction
	def getodrv0_clicked(self):
		self.lvbus_voltage.setText(str(my_drive.vbus_voltage))
		self.lserial_number.setText(str(my_drive.serial_number))
		self.lhw_version_major.setText(str(my_drive.hw_version_minor))
		self.lhw_version_minor.setText(str(my_drive.hw_version_major))
		self.lhw_version_variant.setText(str(my_drive.hw_version_variant))
		self.lfw_version_major.setText(str(my_drive.fw_version_major))
		self.lfw_version_minor.setText(str(my_drive.fw_version_minor))
		self.lfw_version_revision.setText(str(my_drive.fw_version_revision))
		self.lfw_version_unreleased.setText(str(my_drive.fw_version_unreleased))
		self.luser_config_loaded.setText(str(my_drive.user_config_loaded))
		self.lbrake_resistor_armed.setText(str(my_drive.brake_resistor_armed))

		self.lbrake_resistance.setText(str(my_drive.config.brake_resistance))
		self.lenable_uart.setText(str(my_drive.config.enable_uart))
		self.lenable_i2c_instead_of_can.setText(str(my_drive.config.enable_i2c_instead_of_can))
		self.lenable_ascii_protocol_on_usb.setText(str(my_drive.config.enable_ascii_protocol_on_usb))
		self.ldc_bus_undervoltage_trip_level.setText(str(my_drive.config.dc_bus_undervoltage_trip_level))
		self.ldc_bus_overvoltage_trip_level.setText(str(my_drive.config.dc_bus_overvoltage_trip_level))

	#Get Axis param fuction
	def getaxis_clicked(self):
		if self.rbaxis.isChecked() == True:
			print("succes")
			intcs = int(my_drive.axis0.current_state)
			text= self.cbrequested_state.itemText(intcs)
			print(str((intcs))+(" ")+(text))
			self.lcurrentstate.setText(str((intcs))+(" ")+(text))
			self.lerroraxis.setText(str(hex(my_drive.axis0.error)))
			self.lgettemp.setText(str(my_drive.axis0.get_temp()))
			self.lloopcounter.setText(str(my_drive.axis0.loop_counter))

			self.lstartup_motor_calibration.setText(str(my_drive.axis0.config.startup_motor_calibration))
			self.lstartup_encoder_index_search.setText(str(my_drive.axis0.config.startup_encoder_index_search))
			self.lstartup_encoder_offset_calibration.setText(str(my_drive.axis0.config.startup_encoder_offset_calibration))
			self.lstartup_closed_loop_control.setText(str(my_drive.axis0.config.startup_closed_loop_control))
			self.lstartup_sensorless_control.setText(str(my_drive.axis0.config.startup_sensorless_control))
			self.lcounts_per_step.setText(str(my_drive.axis0.config.counts_per_step))
			self.lramp_up_time.setText(str(my_drive.axis0.config.ramp_up_time))
			self.lramp_up_distance.setText(str(my_drive.axis0.config.ramp_up_distance))
			self.lspin_up_current.setText(str(my_drive.axis0.config.spin_up_current))
			self.lspin_up_acceleration.setText(str(my_drive.axis0.config.spin_up_acceleration))
			self.lspin_up_target_vel.setText(str(my_drive.axis0.config.spin_up_target_vel))
			#self.lenablemotorpin.setText(str(my_drive.axis0.config.enable_motor_pin))
			self.lenablestepdir.setText(str(my_drive.axis0.config.enable_step_dir))
		else :
			intcs = int(my_drive.axis1.current_state)
			text= self.cbrequested_state.itemText(intcs)
			self.lcurrentstate.setText(str((intcs))+(" ")+(text))
			#self.lenablemotorpin.setText(str(my_drive.axis1.config.enable_motor_pin))
			self.lenablestepdir.setText(str(my_drive.axis1.config.enable_step_dir))
			self.lerroraxis.setText(str(hex(my_drive.axis1.error)))
			self.lgettemp.setText(str(my_drive.axis1.get_temp()))
			self.lloopcounter.setText(str(my_drive.axis1.loop_counter))

			self.lstartup_motor_calibration.setText(str(my_drive.axis1.config.startup_motor_calibration))
			self.lstartup_encoder_index_search.setText(str(my_drive.axis1.config.startup_encoder_index_search))
			self.lstartup_encoder_offset_calibration.setText(str(my_drive.axis1.config.startup_encoder_offset_calibration))
			self.lstartup_closed_loop_control.setText(str(my_drive.axis1.config.startup_closed_loop_control))
			self.lstartup_sensorless_control.setText(str(my_drive.axis1.config.startup_sensorless_control))
			self.lcounts_per_step.setText(str(my_drive.axis1.config.counts_per_step))
			self.lramp_up_time.setText(str(my_drive.axis1.config.ramp_up_time))
			self.lramp_up_distance.setText(str(my_drive.axis1.config.ramp_up_distance))
			self.lspin_up_current.setText(str(my_drive.axis1.config.spin_up_current))
			self.lspin_up_acceleration.setText(str(my_drive.axis1.config.spin_up_acceleration))
			self.lspin_up_target_vel.setText(str(my_drive.axis1.config.spin_up_target_vel))


	#Get Motor param fuction
	def getmotor_clicked(self):
		if self.rbaxis.isChecked() == True:
			self.lmotor_error.setText(str(hex(my_drive.axis0.motor.error)))
			self.larmed_state.setText(str(my_drive.axis0.motor.armed_state))
			self.lis_calibrated.setText(str(my_drive.axis0.motor.is_calibrated))
			self.lcurrent_meas_phB.setText(str(my_drive.axis0.motor.current_meas_phB))
			self.lcurrent_meas_phC.setText(str(my_drive.axis0.motor.current_meas_phC))
			self.lDC_calib_phB.setText(str(my_drive.axis0.motor.DC_calib_phB))
			self.lDC_calib_phC.setText(str(my_drive.axis0.motor.DC_calib_phC))
			self.lphase_current_rev_gain.setText(str(my_drive.axis0.motor.phase_current_rev_gain))


			self.lp_gain.setText(str(my_drive.axis0.motor.current_control.p_gain))
			self.li_gain.setText(str(my_drive.axis0.motor.current_control.i_gain))
			self.lv_current_control_integral_d.setText(str(my_drive.axis0.motor.current_control.v_current_control_integral_d))
			self.lv_current_control_integral_q.setText(str(my_drive.axis0.motor.current_control.v_current_control_integral_q))
			self.lIbus.setText(str(my_drive.axis0.motor.current_control.Ibus))
			self.lfinal_v_alpha.setText(str(my_drive.axis0.motor.current_control.final_v_alpha))
			self.lfinal_v_beta.setText(str(my_drive.axis0.motor.current_control.final_v_beta))
			self.lIq_setpoint.setText(str(my_drive.axis0.motor.current_control.Iq_setpoint))
			self.lIq_measured.setText(str(my_drive.axis0.motor.current_control.Iq_measured))
			self.lmax_allowed_current.setText(str(my_drive.axis0.motor.current_control.max_allowed_current))


			self.lpre_calibrated_2.setText(str(my_drive.axis0.motor.config.pre_calibrated))
			self.lpole_pairs.setText(str(my_drive.axis0.motor.config.pole_pairs))
			self.lcalibration_current.setText(str(my_drive.axis0.motor.config.calibration_current))
			self.lresistance_calib_max_voltage.setText(str(my_drive.axis0.motor.config.resistance_calib_max_voltage))
			self.lphase_inductance.setText(str(my_drive.axis0.motor.config.phase_inductance))
			self.lphase_resistance.setText(str(my_drive.axis0.motor.config.phase_resistance))
			self.ldirection.setText(str(my_drive.axis0.motor.config.direction))
			intcs = int(my_drive.axis0.motor.config.motor_type)
			text= self.cbmotor_type.itemText(intcs)
			print(str((intcs))+(" ")+(text))
			self.lmotor_type.setText(str((intcs))+(" ")+(text))
			self.lcurrent_lim.setText(str(my_drive.axis0.motor.config.current_lim))
			self.lrequested_current_range.setText(str(my_drive.axis0.motor.config.requested_current_range))
			self.lcurrent_control_bandwidth.setText(str(my_drive.axis0.motor.config.current_control_bandwidth))

		else :
			self.lmotor_error.setText(str(hex(my_drive.axis1.motor.error)))
			self.larmed_state.setText(str(my_drive.axis1.motor.armed_state))
			self.lis_calibrated.setText(str(my_drive.axis1.motor.is_calibrated))
			self.lcurrent_meas_phB.setText(str(my_drive.axis1.motor.current_meas_phB))
			self.lcurrent_meas_phC.setText(str(my_drive.axis1.motor.current_meas_phC))
			self.lDC_calib_phB.setText(str(my_drive.axis1.motor.DC_calib_phB))
			self.lDC_calib_phC.setText(str(my_drive.axis1.motor.DC_calib_phC))
			self.lphase_current_rev_gain.setText(str(my_drive.axis1.motor.phase_current_rev_gain))


			self.lp_gain.setText(str(my_drive.axis1.motor.current_control.p_gain))
			self.li_gain.setText(str(my_drive.axis1.motor.current_control.i_gain))
			self.lv_current_control_integral_d.setText(str(my_drive.axis1.motor.current_control.v_current_control_integral_d))
			self.lv_current_control_integral_q.setText(str(my_drive.axis1.motor.current_control.v_current_control_integral_q))
			self.lIbus.setText(str(my_drive.axis1.motor.current_control.Ibus))
			self.lfinal_v_alpha.setText(str(my_drive.axis1.motor.current_control.final_v_alpha))
			self.lfinal_v_beta.setText(str(my_drive.axis1.motor.current_control.final_v_beta))
			self.lIq_setpoint.setText(str(my_drive.axis1.motor.current_control.Iq_setpoint))
			self.lIq_measured.setText(str(my_drive.axis1.motor.current_control.Iq_measured))
			self.lmax_allowed_current.setText(str(my_drive.axis1.motor.current_control.max_allowed_current))


			self.lpre_calibrated_2.setText(str(my_drive.axis1.motor.config.pre_calibrated))
			self.lpole_pairs.setText(str(my_drive.axis1.motor.config.pole_pairs))
			self.lcalibration_current.setText(str(my_drive.axis1.motor.config.calibration_current))
			self.lresistance_calib_max_voltage.setText(str(my_drive.axis1.motor.config.resistance_calib_max_voltage))
			self.lphase_inductance.setText(str(my_drive.axis1.motor.config.phase_inductance))
			self.lphase_resistance.setText(str(my_drive.axis1.motor.config.phase_resistance))
			self.ldirection.setText(str(my_drive.axis1.motor.config.direction))
			intcs = int(my_drive.axis1.motor.config.motor_type)
			text= self.cbmotor_type.itemText(intcs)
			print(str((intcs))+(" ")+(text))
			self.lmotor_type.setText(str((intcs))+(" ")+(text))
			self.lcurrent_lim.setText(str(my_drive.axis1.motor.config.current_lim))
			self.lrequested_current_range.setText(str(my_drive.axis1.motor.config.requested_current_range))
			self.lcurrent_control_bandwidth.setText(str(my_drive.axis1.motor.config.current_control_bandwidth))


	#Get Controller param fuction
	def getcontroller_clicked(self):
		if self.rbaxis.isChecked() == True:
			self.lpos_setpoint.setText(str(my_drive.axis0.controller.pos_setpoint))
			self.lvel_setpoint.setText(str(my_drive.axis0.controller.vel_setpoint))
			self.lcurrent_setpoint.setText(str(my_drive.axis0.controller.current_setpoint))
			self.lvel_integrator_current.setText(str(my_drive.axis0.controller.vel_integrator_current))
			self.lcurrent_setpoint.setText(str(my_drive.axis0.controller.current_setpoint))

			intcs = int(my_drive.axis0.controller.config.control_mode)
			text= self.cbcontrol_mode.itemText(intcs)
			print(str((intcs))+(" ")+(text))
			self.lcontrol_mode.setText(str((intcs))+(" ")+(text))
			self.lpos_gain.setText(str(my_drive.axis0.controller.config.pos_gain))
			self.lvel_gain.setText(str(my_drive.axis0.controller.config.vel_gain))
			self.lvel_integrator_gain.setText(str(my_drive.axis0.controller.config.vel_integrator_gain))
			self.lvel_limit.setText(str(my_drive.axis0.controller.config.vel_limit))
			self.lenable_gain_scheduling.setText(str(my_drive.axis0.controller.config.enable_gain_scheduling))
			self.lgain_scheduling_width.setText(str(my_drive.axis0.controller.config.gain_scheduling_width))

		else :
			self.lpos_setpoint.setText(str(my_drive.axis1.controller.pos_setpoint))
			self.lvel_setpoint.setText(str(my_drive.axis1.controller.vel_setpoint))
			self.lcurrent_setpoint.setText(str(my_drive.axis1.controller.current_setpoint))
			self.lvel_integrator_current.setText(str(my_drive.axis1.controller.vel_integrator_current))
			self.lcurrent_setpoint.setText(str(my_drive.axis1.controller.current_setpoint))

			intcs = int(my_drive.axis1.controller.config.control_mode)
			text= self.cbcontrol_mode.itemText(intcs)
			print(str((intcs))+(" ")+(text))
			self.lcontrol_mode.setText(str((intcs))+(" ")+(text))
			self.lpos_gain.setText(str(my_drive.axis1.controller.config.pos_gain))
			self.lvel_gain.setText(str(my_drive.axis1.controller.config.vel_gain))
			self.lvel_integrator_gain.setText(str(my_drive.axis1.controller.config.vel_integrator_gain))
			self.lvel_limit.setText(str(my_drive.axis1.controller.config.vel_limit))
			self.lenable_gain_scheduling.setText(str(my_drive.axis1.controller.config.enable_gain_scheduling))
			self.lgain_scheduling_width.setText(str(my_drive.axis1.controller.config.gain_scheduling_width))


	#Get Encoder param fuction
	def getencoder_clicked(self):
		if self.rbaxis.isChecked() == True:
			self.lencoder_error.setText(str(hex(my_drive.axis0.encoder.error)))
			self.lencoder_is_ready.setText(str(my_drive.axis0.encoder.is_ready))
			self.lindex_found.setText(str(my_drive.axis0.encoder.index_found))
			self.lshadow_count.setText(str(my_drive.axis0.encoder.shadow_count))
			self.lcount_in_cpr.setText(str(my_drive.axis0.encoder.count_in_cpr))
			self.linterpolation.setText(str(my_drive.axis0.encoder.interpolation))
			self.lphase.setText(str(my_drive.axis0.encoder.phase))
			self.lpos_estimate.setText(str(my_drive.axis0.encoder.pos_estimate))
			self.lpos_cpr.setText(str(my_drive.axis0.encoder.pos_cpr))
			self.lhall_state.setText(str(my_drive.axis0.encoder.hall_state))
			self.lvel_estimate.setText(str(my_drive.axis0.encoder.vel_estimate))

			intcs = int(my_drive.axis0.encoder.config.mode)
			text= self.cbencoder_mode.itemText(intcs)
			print(str((intcs))+(" ")+(text))
			self.lencoder_mode.setText(str((intcs))+(" ")+(text))
			self.luse_index.setText(str(my_drive.axis0.encoder.config.use_index))
			self.lpre_calibrated.setText(str(my_drive.axis0.encoder.config.pre_calibrated))
			self.lidx_search_speed.setText(str(my_drive.axis0.encoder.config.idx_search_speed))
			self.lcpr.setText(str(my_drive.axis0.encoder.config.cpr))
			self.loffset.setText(str(my_drive.axis0.encoder.config.offset))
			self.loffset_float.setText(str(my_drive.axis0.encoder.config.offset_float))
			self.lbandwidth.setText(str(my_drive.axis0.encoder.config.bandwidth))
			self.lcalib_range.setText(str(my_drive.axis0.encoder.config.calib_range))
		else :
			self.lencoder_error.setText(str(hex(my_drive.axis1.encoder.error)))
			self.lencoder_is_ready.setText(str(my_drive.axis1.encoder.is_ready))
			self.lindex_found.setText(str(my_drive.axis1.encoder.index_found))
			self.lshadow_count.setText(str(my_drive.axis1.encoder.shadow_count))
			self.lcount_in_cpr.setText(str(my_drive.axis1.encoder.count_in_cpr))
			self.linterpolation.setText(str(my_drive.axis1.encoder.interpolation))
			self.lphase.setText(str(my_drive.axis1.encoder.phase))
			self.lpos_estimate.setText(str(my_drive.axis1.encoder.pos_estimate))
			self.lpos_cpr.setText(str(my_drive.axis1.encoder.pos_cpr))
			self.lhall_state.setText(str(my_drive.axis1.encoder.hall_state))
			self.lvel_estimate.setText(str(my_drive.axis1.encoder.vel_estimate))

			intcs = int(my_drive.axis0.encoder.config.mode)
			text= self.cbencoder_mode.itemText(intcs)
			print(str((intcs))+(" ")+(text))
			self.lencoder_mode.setText(str((intcs))+(" ")+(text))
			self.luse_index.setText(str(my_drive.axis1.encoder.config.use_index))
			self.lpre_calibrated.setText(str(my_drive.axis1.encoder.config.pre_calibrated))
			self.lidx_search_speed.setText(str(my_drive.axis1.encoder.config.idx_search_speed))
			self.lcpr.setText(str(my_drive.axis1.encoder.config.cpr))
			self.loffset.setText(str(my_drive.axis1.encoder.config.offset))
			self.loffset_float.setText(str(my_drive.axis1.encoder.config.offset_float))
			self.lbandwidth.setText(str(my_drive.axis1.encoder.config.bandwidth))
			self.lcalib_range.setText(str(my_drive.axis1.encoder.config.calib_range))

	#Get Sensorless estimator param fuction
	def getsensorless_estimator_clicked(self):
		if self.rbaxis.isChecked() == True:
			self.lsensorless_estimatorerror.setText(str(hex(my_drive.axis0.sensorless_estimator.error)))
			self.lsensorless_estimatorphase.setText(str(my_drive.axis0.sensorless_estimator.phase))
			self.lpll_pos.setText(str(my_drive.axis0.sensorless_estimator.pll_pos))
			self.lvel_estimate.setText(str(my_drive.axis0.sensorless_estimator.vel_estimate))
			self.lobserver_gain.setText(str(my_drive.axis0.sensorless_estimator.config.observer_gain))
			self.lpll_bandwidth.setText(str(my_drive.axis0.sensorless_estimator.config.pll_bandwidth))
			self.lpm_flux_linkage.setText(str(my_drive.axis0.sensorless_estimator.config.pm_flux_linkage))
		else :
			self.lsensorless_estimatorerror.setText(str(hex(my_drive.axis1.sensorless_estimator.error)))
			self.lsensorless_estimatorphase.setText(str(my_drive.axis1.sensorless_estimator.phase))
			self.lpll_pos.setText(str(my_drive.axis1.sensorless_estimator.pll_pos))
			self.lvel_estimate.setText(str(my_drive.axis1.sensorless_estimator.vel_estimate))
			self.lobserver_gain.setText(str(my_drive.axis1.sensorless_estimator.config.observer_gain))
			self.lpll_bandwidth.setText(str(my_drive.axis1.sensorless_estimator.config.pll_bandwidth))
			self.lpm_flux_linkage.setText(str(my_drive.axis1.sensorless_estimator.config.pm_flux_linkage))		

	#All fuction for all set button
	#Set button off Odrv0 tab
	def brake_resistance_clicked(self):
		text = float(self.lebrake_resistance.text())
		print(text)
		my_drive.brake_resistance = text
		time.sleep(0.5)
		self.getodrv0_clicked()

	def enable_uart_clicked(self):
		text = int(self.cbenable_uart.currentIndex())
		print(text)
		my_drive.enable_uart = text
		time.sleep(0.5)
		self.getodrv0_clicked()

	def enable_i2c_instead_of_can_clicked(self):
		text = int(self.cbenable_i2c_instead_of_can.currentIndex())
		print(text)
		my_drive.enable_i2c_instead_of_can = text
		time.sleep(0.5)
		self.getodrv0_clicked()

	def enable_ascii_protocol_on_usb_clicked(self):
		text = int(self.cbenable_ascii_protocol_on_usb.currentIndex())
		print(text)
		my_drive.enable_ascii_protocol_on_usb = text
		time.sleep(0.5)
		self.getodrv0_clicked()

	def dc_bus_undervoltage_trip_level_clicked(self):
		text = float(self.ledc_bus_undervoltage_trip_level.text())
		print(text)
		my_drive.dc_bus_undervoltage_trip_level= text
		time.sleep(0.5)
		self.getodrv0_clicked()

	def dc_bus_overvoltage_trip_level_clicked(self):
		text = float(self.ledc_bus_overvoltage_trip_level.text())
		print(text)
		my_drive.dc_bus_overvoltage_trip_level= text
		time.sleep(0.5)
		self.getodrv0_clicked()


	#Set button off Axis tab
	def requested_state_clicked(self):
		text = int(self.cbcbrequested_state.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.requested_state = text
		else :
			my_drive.axis1.requested_state = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def axiserror0_clicked(self):
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.error = 0
		else :
			my_drive.axis1.error = 0
		time.sleep(0.5)
		self.getaxis_clicked()
	#Set button off Axis tab(config)
	"""
	def setenablemotorpin_clicked(self):
		text = int(self.cbenablemotorpin.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.enable_motor_pin = text
		else :
			my_drive.axis1.config.enable_motor_pin = text
		time.sleep(0.5)
		self.getaxis_clicked()
	"""

	def setenablestepdir_clicked(self):
		text = int(self.cbenablestepdir.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.enable_step_dir = text
		else :
			my_drive.axis1.config.enable_step_dir = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def startup_motor_calibration_clicked(self):
		text = int(self.cbstartup_motor_calibration.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.startup_motor_calibration = text
		else :
			my_drive.axis1.config.startup_motor_calibration = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def startup_encoder_index_search_clicked(self):
		text = int(self.cbstartup_encoder_index_search.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.startup_encoder_index_search = text
		else :
			my_drive.axis1.config.startup_encoder_index_search = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def startup_encoder_offset_calibration_clicked(self):
		text = int(self.cbstartup_encoder_offset_calibration.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.startup_encoder_offset_calibration = text
		else :
			my_drive.axis1.config.startup_encoder_offset_calibration = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def startup_closed_loop_control_clicked(self):
		text = int(self.pbstartup_closed_loop_control.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.startup_closed_loop_control = text
		else :
			my_drive.axis1.config.startup_closed_loop_control = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def startup_sensorless_control_clicked(self):
		text = int(self.startup_sensorless_control.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.startup_sensorless_control = text
		else :
			my_drive.axis1.config.startup_sensorless_control = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def counts_per_step_clicked(self):
		text = float(self.lecounts_per_step.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.startup_sensorless_control = text
		else :
			my_drive.axis1.config.startup_sensorless_control = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def ramp_up_time_clicked(self):
		text = float(self.leramp_up_time.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.ramp_up_time = text
		else :
			my_drive.axis1.config.ramp_up_time = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def ramp_up_distance_clicked(self):
		text = float(self.leramp_up_distance.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.ramp_up_distance = text
		else :
			my_drive.axis1.config.ramp_up_distance = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def spin_up_current_clicked(self):
		text = float(self.lespin_up_current.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.spin_up_current = text
		else :
			my_drive.axis1.config.spin_up_current = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def spin_up_acceleration_clicked(self):
		text = float(self.lespin_up_acceleration.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.spin_up_acceleration = text
		else :
			my_drive.axis1.config.spin_up_acceleration = text
		time.sleep(0.5)
		self.getaxis_clicked()

	def spin_up_target_vel_clicked(self):
		text = float(self.lespin_up_target_vel.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.config.spin_up_target_vel = text
		else :
			my_drive.axis1.config.spin_up_target_vel = text
		time.sleep(0.5)
		self.getaxis_clicked()


	#Set button off Motor tab
	def motorerror0_clicked(self):
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.error = 0
		else :
			my_drive.axis1.motor.error = 0
		time.sleep(0.5)
		self.getmotor_clicked()
	#Set button off Motor tab(config)
	def pre_calibrated_2_clicked(self):
		text = int(self.cbpre_calibrated.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.config.pre_calibrated = text
		else :
			my_drive.axis1.motor.config.pre_calibrated = text
		time.sleep(0.5)
		self.getmotor_clicked()

	def direction_clicked(self):
		text = int(self.cbdirection.text())
		print(text)
		"""
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.config.direction = text
		else :
			my_drive.axis1.motor.config.direction = text
		time.sleep(0.5)
		self.getmotor_clicked()
		"""

	def motor_type_clicked(self):
		text = int(self.motor_type.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.config.motor_type = text
		else :
			my_drive.axis1.motor.config.motor_type = text
		time.sleep(0.5)
		self.getmotor_clicked()

	def pole_pairs_clicked(self):
		text = int(self.lepole_pairs.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.config.pole_pairs = text
		else :
			my_drive.axis1.motor.config.pole_pairs = text
		time.sleep(0.5)
		self.getmotor_clicked()
	def calibration_current_clicked(self):
		text = float(self.lecalibration_current.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.config.calibration_current = text
		else :
			my_drive.axis1.motor.config.calibration_current = text
		time.sleep(0.5)
		self.getmotor_clicked()

	def resistance_calib_max_voltage_clicked(self):
		text = float(self.leresistance_calib_max_voltage.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.config.resistance_calib_max_voltage = text
		else :
			my_drive.axis1.motor.config.resistance_calib_max_voltage = text
		time.sleep(0.5)
		self.getmotor_clicked()

	def phase_inductance_clicked(self):
		text = float(self.lephase_inductance.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.config.phase_inductance = text
		else :
			my_drive.axis1.motor.config.phase_inductance = text
		time.sleep(0.5)
		self.getmotor_clicked()

	def phase_resistance_clicked(self):
		text = float(self.lephase_resistance.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.config.phase_resistance = text
		else :
			my_drive.axis1.motor.config.phase_resistance = text
		time.sleep(0.5)
		self.getmotor_clicked()

	def current_lim_clicked(self):
		text = float(self.lecurrent_lim.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.config.current_lim = text
		else :
			my_drive.axis1.motor.config.current_lim = text
		time.sleep(0.5)
		self.getmotor_clicked()

	def requested_current_range_clicked(self):
		text = float(self.requested_current_range.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.config.requested_current_range = text
		else :
			my_drive.axis1.motor.config.requested_current_range = text
		time.sleep(0.5)
		self.getmotor_clicked()

	def current_control_bandwidth_clicked(self):
		text = float(self.lecurrent_control_bandwidth.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.motor.config.current_control_bandwidth = text
		else :
			my_drive.axis1.motor.config.current_control_bandwidth = text
		time.sleep(0.5)
		self.getmotor_clicked()

	#Set button off Controller tab(config)
	def control_mode_clicked(self):
		text = int(self.cbcontrol_mode.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.controller.config.control_mode = text
		else :
			my_drive.axis1.controller.config.control_mode = text
		time.sleep(0.5)
		self.getcontroller_clicked()

	def pos_gain_clicked(self):
		text = float(self.lepos_gain.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.controller.config.pos_gain = text
		else :
			my_drive.axis1.controller.config.pos_gain = text
		time.sleep(0.5)
		self.getcontroller_clicked()

	def vel_gain_clicked(self):
		text = float(self.level_gain.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.controller.config.vel_gain = text
		else :
			my_drive.axis1.controller.config.vel_gain = text
		time.sleep(0.5)
		self.getcontroller_clicked()

	def vel_integrator_gain_clicked(self):
		text = float(self.level_integrator_gain.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.controller.config.vel_integrator_gain = text
		else :
			my_drive.axis1.controller.config.vel_integrator_gain = text
		time.sleep(0.5)
		self.getcontroller_clicked()

	def vel_limit_clicked(self):
		text = float(self.level_limit.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.controller.config.vel_limit = text
		else :
			my_drive.axis1.controller.config.vel_limit = text
		time.sleep(0.5)
		self.getcontroller_clicked()

	def enable_gain_scheduling_clicked(self):
		text = int(self.cbenable_gain_scheduling.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.controller.config.enable_gain_scheduling = text
		else :
			my_drive.axis1.controller.config.enable_gain_scheduling = text
		time.sleep(0.5)
		self.getcontroller_clicked()

	def gain_scheduling_width_clicked(self):
		text = float(self.legain_scheduling_width.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.controller.config.gain_scheduling_width = text
		else :
			my_drive.axis1.controller.config.gain_scheduling_width = text
		time.sleep(0.5)
		self.getcontroller_clicked()

	#Set button off Encoder tab
	def encodererror0_clicked(self):
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.encoder.error = 0
		else :
			my_drive.axis1.encoder.error = 0
		time.sleep(0.5)
		self.getencoder_clicked()
	#Set button off Encoder tab(config)
	def mode_clicked(self):
		text = int(self.cbmode.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.encoder.config.mode = text
		else :
			my_drive.axis1.encoder.config.mode = text 
		time.sleep(0.5)
		self.getencoder_clicked()

	def use_index_clicked(self):
		text = int(self.cbuse_index.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.encoder.config.use_index = text
		else :
			my_drive.axis1.encoder.config.use_index = text
		time.sleep(0.5)
		self.getencoder_clicked()

	def pre_calibrated_clicked(self):
		text = int(self.cbpre_calibrated.currentIndex())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.encoder.config.pre_calibrated = text
		else :
			my_drive.axis1.encoder.config.pre_calibrated = text
		time.sleep(0.5)
		self.getencoder_clicked()

	def idx_search_speed_clicked(self):
		text = float(self.leidx_search_speed.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.encoder.config.idx_search_speed = text
		else :
			my_drive.axis1.encoder.config.idx_search_speed = text
		time.sleep(0.5)
		self.getencoder_clicked()

	def cpr_clicked(self):
		text = int(self.lecpr.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.encoder.config.cpr = text
		else :
			my_drive.axis1.encoder.config.cpr = text
		time.sleep(0.5)
		self.getencoder_clicked()

	def offset_clicked(self):
		text = int(self.leoffset.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.encoder.config.offset = text
		else :
			my_drive.axis1.encoder.config.offset = text
		time.sleep(0.5)
		self.getencoder_clicked()

	def offset_float_clicked(self):
		text = float(self.leoffset_float.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.encoder.config.offset_float = text
		else :
			my_drive.axis1.encoder.config.offset_float = text
		time.sleep(0.5)
		self.getencoder_clicked()

	def bandwidth_clicked(self):
		text = float(self.lebandwidth.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.encoder.config.bandwidth = text
		else :
			my_drive.axis1.encoder.config.bandwidth = text
		time.sleep(0.5)
		self.getencoder_clicked()

	def calib_range_clicked(self):
		text = float(self.lecalib_range.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.encoder.config.calib_range = text
		else :
			my_drive.axis1.encoder.config.calib_range = text
		time.sleep(0.5)
		self.getencoder_clicked()


	#Set button off Sensorless tab(config)
	def sensorless_estimator_error_clicked(self):
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.sensorless_estimator = 0
		else :
			my_drive.axis1.sensorless_estimator = 0
		time.sleep(0.5)
		self.getsensorless_estimator_clicked()

	def observer_gain_clicked(self):
		text = float(self.leobserver_gain.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.sensorless_estimator.config.observer_gain = text
		else :
			my_drive.axis1.sensorless_estimator.config.observer_gain = text
		time.sleep(0.5)
		self.getsensorless_estimator_clicked()

	def pll_bandwidth_clicked(self):
		text = float(self.lepll_bandwidth.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.sensorless_estimator.config.pll_bandwidth = text
		else :
			my_drive.axis1.sensorless_estimator.config.pll_bandwidth = text
		time.sleep(0.5)
		self.getsensorless_estimator_clicked()

	def pm_flux_linkage_clicked(self):
		text = float(self.lepm_flux_linkage.text())
		print(text)
		if self.rbaxis.isChecked() == True:
			my_drive.axis0.sensorless_estimator.config.pm_flux_linkage = text
		else :
			my_drive.axis1.sensorless_estimator.config.pm_flux_linkage = text
		time.sleep(0.5)
		self.getsensorless_estimator_clicked()

		
app=QApplication(sys.argv)
widget=odriveUI()
widget.show()
sys.exit(app.exec())




