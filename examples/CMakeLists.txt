###################################################################
#
# gnssLab examples
#
###################################################################

add_executable(parse_opt exam-1.1-parse_opt.cpp)
target_link_libraries(parse_opt PRIVATE gnss)

add_executable(parse_config exam-1.2-parse_config.cpp)
target_link_libraries(parse_config PRIVATE gnss)

add_executable(eigen exam-2.1-eigen.cpp)
target_link_libraries(eigen PRIVATE gnss)

add_executable(time_convert exam-2.2-time_convert.cpp)
target_link_libraries(time_convert PRIVATE gnss)

add_executable(coord_myself exam-2.3-myself.cpp)
target_link_libraries(coord_myself PRIVATE gnss)

# Legacy coordinate-conversion examples remain disabled.
# add_executable(coord_convert1 exam-2.3-coord_convert1.cpp)
# target_link_libraries(coord_convert1 PRIVATE gnss)
#
# add_executable(coord_convert2 exam-2.3-coord_convert2.cpp)
# target_link_libraries(coord_convert2 PRIVATE gnss)
#
# add_executable(coord_convert3 exam-2.3-coord_convert3.cpp)
# target_link_libraries(coord_convert3 PRIVATE gnss)
#
# add_executable(coord_convert4 exam-2.3-coord_convert4.cpp)
# target_link_libraries(coord_convert4 PRIVATE gnss)
#
# add_executable(coord_convert5 exam-2.3-coord_convert5.cpp)
# target_link_libraries(coord_convert5 PRIVATE gnss)

add_executable(skyplot exam-2.4-skyplot.cpp)
target_link_libraries(skyplot PRIVATE gnss)

add_executable(satid exam-3.1-satid.cpp)
target_link_libraries(satid PRIVATE gnss)

add_executable(read_rinex_data exam-3.2-read_rinex_data.cpp)
target_link_libraries(read_rinex_data PRIVATE gnss)

add_executable(obs_statistics exam-3.3-obs-statistics.cpp)
target_link_libraries(obs_statistics PRIVATE gnss)

add_executable(gps_eph exam-4.1-gps_eph.cpp)
target_link_libraries(gps_eph PRIVATE gnss)

add_executable(system_bias exam-5.1-system_bias.cpp)
target_link_libraries(system_bias PRIVATE gnss)

add_executable(atmospheric_delay exam-5.2-atmospheric_delay.cpp)
target_link_libraries(atmospheric_delay PRIVATE gnss)

add_executable(atmospheric_test exam-5.3-spp_atmospheric_test.cpp)
target_link_libraries(atmospheric_test PRIVATE gnss)

add_executable(spp_tgd exam-5.4-tgd_test.cpp)
target_link_libraries(spp_tgd PRIVATE gnss)

add_executable(bds_satweight_test exam-5.5-bds-satweight-test.cpp)
target_link_libraries(bds_satweight_test PRIVATE gnss)

add_executable(spp_error_model_test exam-5.6-spp-error-model-test.cpp)
target_link_libraries(spp_error_model_test PRIVATE gnss)

add_executable(sppif_error_model_test exam-6.6-sppif-error-model-test.cpp)
target_link_libraries(sppif_error_model_test PRIVATE gnss)

add_executable(sppgf_error_model_test exam-6.7-sppgf-error-model-test.cpp)
target_link_libraries(sppgf_error_model_test PRIVATE gnss)

add_executable(sppif exam-6.1-sppif.cpp)
target_link_libraries(sppif PRIVATE gnss)

add_executable(sppUC exam-6.3-GFCode.cpp)
target_link_libraries(sppUC PRIVATE gnss)

add_executable(sppif_bds_test exam-6.4-sppif-bds-test.cpp)
target_link_libraries(sppif_bds_test PRIVATE gnss)

add_executable(bds_ifcode_stat exam-6.5-bds_ifcode_stat.cpp)
target_link_libraries(bds_ifcode_stat PRIVATE gnss)

add_executable(spp_velocity exam-6.6-spp-velocity.cpp)
target_link_libraries(spp_velocity PRIVATE gnss)

# Currently inactive legacy targets remain disabled.
# add_executable(cs_detect_mw exam-7.1-cs_detect_mw.cpp)
# target_link_libraries(cs_detect_mw PRIVATE gnss)
#
# add_executable(lambda exam-8.3-lambda.cpp)
# target_link_libraries(lambda PRIVATE gnss)
#
# add_executable(rtk_lsq exam-8.4-rtk_lsq.cpp)
# target_link_libraries(rtk_lsq PRIVATE gnss)
#
# add_executable(rtk_kal exam-8.5-rtk_kal.cpp)
# target_link_libraries(rtk_kal PRIVATE gnss)

add_executable(diagnostic_bds exam-diagnostic-bds.cpp)
target_link_libraries(diagnostic_bds PRIVATE gnss)

add_executable(diagnostic_iono exam-diagnostic-iono.cpp)
target_link_libraries(diagnostic_iono PRIVATE gnss)

add_executable(diagnostic_tropo exam-diagnostic-tropo.cpp)
target_link_libraries(diagnostic_tropo PRIVATE gnss)

add_executable(multi_spp exam-6.7-multi-spp.cpp)
target_link_libraries(multi_spp PRIVATE gnss)

add_executable(multi_if exam-6.8-multi-if.cpp)
target_link_libraries(multi_if PRIVATE gnss)

add_executable(multi_gf exam-6.9-multi-gf.cpp)
target_link_libraries(multi_gf PRIVATE gnss)

add_executable(diagnostic_if exam-diagnostic-if.cpp)
target_link_libraries(diagnostic_if PRIVATE gnss)

add_executable(bds_c2i_c7i_count exam-bds-c2i-c7i-count.cpp)
target_link_libraries(bds_c2i_c7i_count PRIVATE gnss)

add_executable(quality_analysis exam-quality-analysis.cpp)
target_link_libraries(quality_analysis PRIVATE gnss)
