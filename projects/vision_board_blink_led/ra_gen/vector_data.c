/* generated vector source file - do not edit */
        #include "bsp_api.h"
        /* Do not build these data structures if no interrupts are currently allocated because IAR will have build errors. */
        #if VECTOR_DATA_IRQ_COUNT > 0
        #if __has_include("r_ioport.h")
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_ICU_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
                        [0] = sci_b_uart_rxi_isr, /* SCI2 RXI (Receive data full) */
            [1] = sci_b_uart_txi_isr, /* SCI2 TXI (Transmit data empty) */
            [2] = sci_b_uart_tei_isr, /* SCI2 TEI (Transmit end) */
            [3] = sci_b_uart_eri_isr, /* SCI2 ERI (Receive error) */
            [4] = sci_b_i2c_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [5] = sci_b_i2c_tei_isr, /* SCI9 TEI (Transmit end) */
            [6] = glcdc_line_detect_isr, /* GLCDC LINE DETECT (Specified line) */
            [7] = mipi_dsi_seq0, /* DSI SEQ0 */
            [8] = mipi_dsi_seq1, /* DSI SEQ1 */
            [9] = mipi_dsi_vin1, /* DSI VIN1 */
            [10] = mipi_dsi_rcv, /* DSI RCV */
            [11] = mipi_dsi_ferr, /* DSI FERR */
            [12] = mipi_dsi_ppi, /* DSI PPI */
            [13] = drw_int_isr, /* DRW INT (DRW interrupt) */
        };
        const bsp_interrupt_event_t g_interrupt_event_link_select[BSP_ICU_VECTOR_MAX_ENTRIES] =
        {
            [0] = BSP_PRV_IELS_ENUM(EVENT_SCI2_RXI), /* SCI2 RXI (Receive data full) */
            [1] = BSP_PRV_IELS_ENUM(EVENT_SCI2_TXI), /* SCI2 TXI (Transmit data empty) */
            [2] = BSP_PRV_IELS_ENUM(EVENT_SCI2_TEI), /* SCI2 TEI (Transmit end) */
            [3] = BSP_PRV_IELS_ENUM(EVENT_SCI2_ERI), /* SCI2 ERI (Receive error) */
            [4] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TXI), /* SCI9 TXI (Transmit data empty) */
            [5] = BSP_PRV_IELS_ENUM(EVENT_SCI9_TEI), /* SCI9 TEI (Transmit end) */
            [6] = BSP_PRV_IELS_ENUM(EVENT_GLCDC_LINE_DETECT), /* GLCDC LINE DETECT (Specified line) */
            [7] = BSP_PRV_IELS_ENUM(EVENT_MIPI_DSI_SEQ0), /* DSI SEQ0 */
            [8] = BSP_PRV_IELS_ENUM(EVENT_MIPI_DSI_SEQ1), /* DSI SEQ1 */
            [9] = BSP_PRV_IELS_ENUM(EVENT_MIPI_DSI_VIN1), /* DSI VIN1 */
            [10] = BSP_PRV_IELS_ENUM(EVENT_MIPI_DSI_RCV), /* DSI RCV */
            [11] = BSP_PRV_IELS_ENUM(EVENT_MIPI_DSI_FERR), /* DSI FERR */
            [12] = BSP_PRV_IELS_ENUM(EVENT_MIPI_DSI_PPI), /* DSI PPI */
            [13] = BSP_PRV_IELS_ENUM(EVENT_DRW_INT), /* DRW INT (DRW interrupt) */
        };
        #elif __has_include("r_ioport_b.h")
        BSP_DONT_REMOVE const fsp_vector_t g_vector_table[BSP_IRQ_VECTOR_MAX_ENTRIES] BSP_PLACE_IN_SECTION(BSP_SECTION_APPLICATION_VECTORS) =
        {
            [BSP_PRV_IELS_ENUM(SCI2_RXI)] = sci_b_uart_rxi_isr, /* SCI2 RXI (Receive data full) */
            [BSP_PRV_IELS_ENUM(SCI2_TXI)] = sci_b_uart_txi_isr, /* SCI2 TXI (Transmit data empty) */
            [BSP_PRV_IELS_ENUM(SCI2_TEI)] = sci_b_uart_tei_isr, /* SCI2 TEI (Transmit end) */
            [BSP_PRV_IELS_ENUM(SCI2_ERI)] = sci_b_uart_eri_isr, /* SCI2 ERI (Receive error) */
            [BSP_PRV_IELS_ENUM(SCI9_TXI)] = sci_b_i2c_txi_isr, /* SCI9 TXI (Transmit data empty) */
            [BSP_PRV_IELS_ENUM(SCI9_TEI)] = sci_b_i2c_tei_isr, /* SCI9 TEI (Transmit end) */
            [BSP_PRV_IELS_ENUM(GLCDC_LINE_DETECT)] = glcdc_line_detect_isr, /* GLCDC LINE DETECT (Specified line) */
            [BSP_PRV_IELS_ENUM(MIPI_DSI_SEQ0)] = mipi_dsi_seq0, /* DSI SEQ0 */
            [BSP_PRV_IELS_ENUM(MIPI_DSI_SEQ1)] = mipi_dsi_seq1, /* DSI SEQ1 */
            [BSP_PRV_IELS_ENUM(MIPI_DSI_VIN1)] = mipi_dsi_vin1, /* DSI VIN1 */
            [BSP_PRV_IELS_ENUM(MIPI_DSI_RCV)] = mipi_dsi_rcv, /* DSI RCV */
            [BSP_PRV_IELS_ENUM(MIPI_DSI_FERR)] = mipi_dsi_ferr, /* DSI FERR */
            [BSP_PRV_IELS_ENUM(MIPI_DSI_PPI)] = mipi_dsi_ppi, /* DSI PPI */
            [BSP_PRV_IELS_ENUM(DRW_INT)] = drw_int_isr, /* DRW INT (DRW interrupt) */
        };
        #endif
        #endif