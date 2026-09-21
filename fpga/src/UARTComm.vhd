library IEEE;
use IEEE.STD_LOGIC_1164.ALL;
use IEEE.NUMERIC_STD.ALL;

entity UARTComm is
    Port (
        clk     : in  std_logic;
        uart_rx : in  std_logic;
        uart_tx : out std_logic;
        led0    : out std_logic;
        led1    : out std_logic
    );
end UARTComm;

architecture Behavioral of UARTComm is
    constant CLK_FREQ   : integer := 125000000;
    constant BAUD_RATE  : integer := 115200;
    constant BIT_TICKS  : integer := CLK_FREQ / BAUD_RATE;
    constant HALF_TICKS : integer := BIT_TICKS / 2;

    type rx_state_t is (IDLE, START_CHECK, DATA_BITS, STOP_BIT);
    signal rx_state : rx_state_t := IDLE;

    signal rx_meta   : std_logic := '1';
    signal rx_sync   : std_logic := '1';

    signal led0_reg  : std_logic := '0';
    signal led1_reg  : std_logic := '0';

    signal tick_count : integer := 0;
    signal bit_index  : integer range 0 to 7 := 0;
    signal rx_shift   : std_logic_vector(7 downto 0) := (others => '0');

    signal tx_busy      : std_logic := '0';
    signal tx_count     : integer := 0;
    signal tx_bit_index : integer range 0 to 9 := 0;
    signal tx_frame     : std_logic_vector(9 downto 0) := (others => '1');
    signal uart_tx_reg  : std_logic := '1';

begin
    led0 <= led0_reg;
    led1 <= led1_reg;
    uart_tx <= uart_tx_reg;

    process(clk)
    begin
        if rising_edge(clk) then
            rx_meta <= uart_rx;
            rx_sync <= rx_meta;

            -- TX
            if tx_busy = '1' then
                led1_reg <= '1';
                uart_tx_reg <= tx_frame(tx_bit_index);

                if tx_count = BIT_TICKS - 1 then
                    tx_count <= 0;

                    if tx_bit_index = 9 then
                        tx_busy <= '0';
                        tx_bit_index <= 0;
                        uart_tx_reg <= '1';
                        led1_reg <= '0';
                    else
                        tx_bit_index <= tx_bit_index + 1;
                    end if;
                else
                    tx_count <= tx_count + 1;
                end if;
            end if;

            -- RX
            case rx_state is
                when IDLE =>
                    tick_count <= 0;
                    bit_index  <= 0;

                    if rx_sync = '0' then
                        rx_state <= START_CHECK;
                    end if;

                when START_CHECK =>
                    if tick_count = HALF_TICKS - 1 then
                        tick_count <= 0;

                        if rx_sync = '0' then
                            rx_state <= DATA_BITS;
                            bit_index <= 0;
                        else
                            rx_state <= IDLE;
                        end if;
                    else
                        tick_count <= tick_count + 1;
                    end if;

                when DATA_BITS =>
                    if tick_count = BIT_TICKS - 1 then
                        tick_count <= 0;
                        rx_shift(bit_index) <= rx_sync;

                        if bit_index = 7 then
                            rx_state <= STOP_BIT;
                        else
                            bit_index <= bit_index + 1;
                        end if;
                    else
                        tick_count <= tick_count + 1;
                    end if;

                when STOP_BIT =>
                    if tick_count = BIT_TICKS - 1 then
                        tick_count <= 0;
                        rx_state <= IDLE;

                        if rx_shift = x"41" then
                            led0_reg <= not led0_reg;

                            -- 'B' full UART frame, bit 0 first:
                            -- start, data LSB-first, stop
                            tx_frame <= "1010000100";
                            tx_busy <= '1';
                            tx_count <= 0;
                            tx_bit_index <= 0;
                        end if;
                    else
                        tick_count <= tick_count + 1;
                    end if;
            end case;
        end if;
    end process;

end Behavioral;