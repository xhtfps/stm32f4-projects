
module ad7606(
	clk						,		                                   
	rst                     ,             
	trig                    ,            
	data_sel                ,              
	ack                     ,              
	data_out                ,              
	ad7606_reset            ,              
	ad7606_convstAB         ,              
	ad7606_cs               ,              
	ad7606_rd               ,              
	ad7606_busy             ,              
	ad7606_frstdata         ,              
	ad7606_data    			,
);
	input             clk                 ;
	input             rst                 ;
	input             trig                ;

	input   [2 : 0]   data_sel            ;

	output            ack                 ;
	output  [15 : 0]  data_out            ;

	output            ad7606_reset        ;
	output            ad7606_convstAB     ;
	output            ad7606_cs           ;
	output            ad7606_rd           ;

	input             ad7606_busy         ;
	input             ad7606_frstdata     ;
	input   [15 : 0]  ad7606_data         ;  


	reg               ad7606_reset        ;
	reg               ad7606_convstAB     ;
	reg               ad7606_cs           ;
	reg               ad7606_rd           ;

	reg               ack                 ;
	wire    [15 : 0]  data_out            ;

	reg     [15 : 0]  databuf1            ;
	reg     [15 : 0]  databuf2            ;
	reg     [15 : 0]  databuf3            ;
	reg     [15 : 0]  databuf4            ;
	reg     [15 : 0]  databuf5            ;
	reg     [15 : 0]  databuf6            ;
	reg     [15 : 0]  databuf7            ;
	reg     [15 : 0]  databuf8            ;
	
	reg     [14 : 0]	state               ;
	
	reg     [15 : 0]  dout_buf            ;
	reg               link_data_out       ; 

	parameter idle         = 15'b000_0000_0000_0001,	
						reset        = 15'b000_0000_0000_0010,
						start        = 15'b000_0000_0000_0100, 
						convert      = 15'b000_0000_0000_1000,
						con_over     = 15'b000_0000_0001_0000,
						start_read   = 15'b000_0000_0010_0000,
						data1        = 15'b000_0000_0100_0000,
						data2        = 15'b000_0000_1000_0000,
						data3        = 15'b000_0001_0000_0000,
						data4        = 15'b000_0010_0000_0000,
						data5        = 15'b000_0100_0000_0000,
						data6        = 15'b000_1000_0000_0000,
						data7        = 15'b001_0000_0000_0000,
						data8        = 15'b010_0000_0000_0000,
						stop         = 15'b100_0000_0000_0000;


	assign data_out = dout_buf ;

	reg               scl                 ;
	
	always @ (posedge clk)
			begin
					if(!trig)
							scl <= 1'b0;
					else
							scl <= clk;
			end		


	reg               data_flag	 		      ;
	
	always @ (posedge clk)
			if(!rst)
					begin						
							databuf1 <= 0;
							databuf2 <= 0;
							databuf3 <= 0;
							databuf4 <= 0;
							databuf5 <= 0;
							databuf6 <= 0;
							databuf7 <= 0;
							databuf8 <= 0;

							ack <= 0;
							link_data_out <= 0;
							state <= idle;
					end
			else
					begin
							case(state)
									idle:
											if(trig == 1)
												begin
															state <= reset;
															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b1;
															ad7606_rd <= 1'b1; 
															ack <= 0;
																															
													end		
											else
													begin	
															state <= idle;
															ack <= 0;
													end
													
									reset:
											if(!trig)
													state <= idle;
											else	
													begin
															state <= start;
															ad7606_reset <= 1'b1;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b1;
															ad7606_rd <= 1'b1; 
															
															link_data_out <= 0;
													end
									start:
											begin
													state <= convert;
													ad7606_reset <= 1'b0;
													ad7606_convstAB <= 1'b0;
													ad7606_cs <= 1'b1;
													ad7606_rd <= 1'b1; 
											end
									convert:
											begin
													state <= con_over;
													ad7606_reset <= 1'b0;
													ad7606_convstAB <= 1'b1;
													ad7606_cs <= 1'b1;
													ad7606_rd <= 1'b1; 
											end									
									con_over:
											if(ad7606_busy)
													state <= con_over;
											else
													begin
															state <= start_read;
															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b1;
															ad7606_rd <= 1'b1; 
													end
									start_read:
											begin
													state <= data1;
													ad7606_reset <= 1'b0;
													ad7606_convstAB <= 1'b1;
													ad7606_cs <= 1'b0;
													ad7606_rd <= 1'b1;
													
													data_flag <= 0; 
											end												
									data1:
											if(!data_flag & (!ad7606_frstdata))
													begin
															data_flag <= 1;
															state <= data1;

															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b0;
													end
											else
													begin							
															data_flag <= 0;						
															state <= data2;
															databuf1 <= ad7606_data; 
															
															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b1;	
													end
									data2:
											if(!data_flag & ad7606_frstdata)
													begin
															data_flag <= 1;
															state <= data2;

															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b0;
													end
											else
													begin							
															data_flag <= 0;						
															state <= data3;
															databuf2 <= ad7606_data; 
															
															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b1;	
													end
									data3:
											if(!data_flag & (!ad7606_frstdata))
													begin
															data_flag <= 1;
															state <= data3;

															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b0;
													end
											else
													begin							
															data_flag <= 0;						
															state <= data4;
															databuf3 <= ad7606_data;
															
															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b1;	
													end
							    data4:
											if(!data_flag & (!ad7606_frstdata))
													begin
															data_flag <= 1;
															state <= data4;

															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b0;
													end
											else
													begin							
															data_flag <= 0;						
															state <= data5;
															databuf4 <= ad7606_data;
															
															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b1;	
													end
									data5:
											if(!data_flag & (!ad7606_frstdata))
													begin
															data_flag <= 1;
															state <= data5;

															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b0;
													end
											else
													begin							
															data_flag <= 0;						
															state <= data6;
															databuf5 <= ad7606_data;
															
															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b1;	
													end
									data6:
											if(!data_flag & (!ad7606_frstdata))
													begin
															data_flag <= 1;
															state <= data6;

															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b0;
													end
											else
													begin							
															data_flag <= 0;						
															state <= data7;
															databuf6 <= ad7606_data;
															
															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b1;	
													end
									data7:
											if(!data_flag & (!ad7606_frstdata))
													begin
															data_flag <= 1;
															state <= data7;

															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b0;
													end
											else
													begin							
															data_flag <= 0;						
															state <= data8;
															databuf7 <= ad7606_data;
															
															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b1;	
													end																						
									data8:
											if(!data_flag & (!ad7606_frstdata))
													begin
															data_flag <= 1;
															state <= data8;

															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b0;
													end
											else
													begin							
															data_flag <= 0;						
															state <= stop;
															databuf8 <= ad7606_data;
															
															ad7606_reset <= 1'b0;
															ad7606_convstAB <= 1'b1;
															ad7606_cs <= 1'b0;
															ad7606_rd <= 1'b1;	
													end												
									stop:
											begin
													state <= idle;
													ad7606_reset <= 1'b0;
													ad7606_convstAB <= 1'b1;
													ad7606_cs <= 1'b1;
													ad7606_rd <= 1'b1;
													ack <= 1'b1;
													link_data_out <= 1'b1;
											end															
									default:
											state <= idle;
							endcase
					end

	always @ (*)
			begin
					case(data_sel)
							3'b000 : dout_buf <= databuf1;
							3'b001 : dout_buf <= databuf2;
							3'b010 : dout_buf <= databuf3;
							3'b011 : dout_buf <= databuf4;
							3'b100 : dout_buf <= databuf5;
							3'b101 : dout_buf <= databuf6;
							3'b110 : dout_buf <= databuf7;
							3'b111 : dout_buf <= databuf8;
							default: dout_buf <= 16'hzzzz;
					endcase
			end			
	
endmodule	