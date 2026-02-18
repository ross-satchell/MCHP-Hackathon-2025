package de.kai_morich.simple_bluetooth_le_terminal;

import android.os.Bundle;
import android.text.Editable;
import android.text.TextWatcher;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.EditText;
import android.widget.SeekBar;
import android.widget.TextView;

import androidx.annotation.NonNull;
import androidx.annotation.Nullable;
import androidx.fragment.app.Fragment;

public class PidTuningFragment extends Fragment {

    private SeekBar pSeekBar, iSeekBar, dSeekBar;
    private TextView pValueTextView, iValueTextView, dValueTextView;
    private EditText pEditText, iEditText, dEditText;
    private Button sendButton, recvButton;

    @Nullable
    @Override
    public View onCreateView(@NonNull LayoutInflater inflater, @Nullable ViewGroup container, @Nullable Bundle savedInstanceState) {
        View view = inflater.inflate(R.layout.fragment_pid_tuning, container, false);

        pSeekBar = view.findViewById(R.id.pid_p_seekbar);
        iSeekBar = view.findViewById(R.id.pid_i_seekbar);
        dSeekBar = view.findViewById(R.id.pid_d_seekbar);

        pValueTextView = view.findViewById(R.id.pid_p_value_textview);
        iValueTextView = view.findViewById(R.id.pid_i_value_textview);
        dValueTextView = view.findViewById(R.id.pid_d_value_textview);

        pEditText = view.findViewById(R.id.pid_p_edittext);
        iEditText = view.findViewById(R.id.pid_i_edittext);
        dEditText = view.findViewById(R.id.pid_d_edittext);

        sendButton = view.findViewById(R.id.pid_send_button);
        recvButton = view.findViewById(R.id.pid_recv_button);

        setupSlider(pSeekBar, pValueTextView, pEditText, 15000);
        setupSlider(iSeekBar, iValueTextView, iEditText, 100);
        setupSlider(dSeekBar, dValueTextView, dEditText, 100);

        sendButton.setOnClickListener(v -> {
            TerminalFragment terminalFragment = (TerminalFragment) getParentFragmentManager().findFragmentByTag("terminal");
            if (terminalFragment != null) {
                String p = pEditText.getText().toString();
                if (p.isEmpty()) p = "0";
                String i = iEditText.getText().toString();
                if (i.isEmpty()) i = "0";
                String d = dEditText.getText().toString();
                if (d.isEmpty()) d = "0";

                terminalFragment.send("P" + formatPidValue(p));
                terminalFragment.send("I" + formatPidValue(i));
                terminalFragment.send("D" + formatPidValue(d));
            }
        });

        recvButton.setOnClickListener(v -> {
            TerminalFragment terminalFragment = (TerminalFragment) getParentFragmentManager().findFragmentByTag("terminal");
            if (terminalFragment != null) {
                terminalFragment.send("GET_PID");
            }
        });

        return view;
    }

    private String formatPidValue(String value) {
        try {
            float floatValue = Float.parseFloat(value);
            if (floatValue == (int) floatValue) {
                return String.valueOf((int) floatValue);
            } else {
                return String.valueOf(floatValue);
            }
        } catch (NumberFormatException e) {
            return value;
        }
    }

    private void setupSlider(SeekBar seekBar, TextView valueTextView, EditText editText, int max) {
        seekBar.setMax(max);
        seekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                if (fromUser) {
                    float value = (float) progress;
                    valueTextView.setText(formatPidValue(String.valueOf(value)));
                    editText.setText(formatPidValue(String.valueOf(value)));
                }
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) { }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) { }
        });

        editText.addTextChangedListener(new TextWatcher() {
            @Override
            public void beforeTextChanged(CharSequence s, int start, int count, int after) { }

            @Override
            public void onTextChanged(CharSequence s, int start, int before, int count) {
                try {
                    float value = Float.parseFloat(s.toString());
                    valueTextView.setText(s);
                    seekBar.setProgress((int) value);
                } catch (NumberFormatException e) {
                    // Ignore
                }
            }

            @Override
            public void afterTextChanged(Editable s) { }
        });
    }
}
