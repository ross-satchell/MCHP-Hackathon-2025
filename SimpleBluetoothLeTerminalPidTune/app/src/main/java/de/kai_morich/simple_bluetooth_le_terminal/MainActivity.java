package de.kai_morich.simple_bluetooth_le_terminal;

import android.content.Context;
import android.content.SharedPreferences;
import android.os.Bundle;
import android.view.MenuItem;
import androidx.appcompat.app.ActionBarDrawerToggle;
import androidx.appcompat.app.AppCompatActivity;
import androidx.appcompat.widget.Toolbar;
import androidx.core.view.GravityCompat;
import androidx.drawerlayout.widget.DrawerLayout;
import androidx.fragment.app.Fragment;
import androidx.fragment.app.FragmentManager;
import com.google.android.material.navigation.NavigationView;

public class MainActivity extends AppCompatActivity implements FragmentManager.OnBackStackChangedListener, NavigationView.OnNavigationItemSelectedListener {

    private DrawerLayout drawerLayout;
    private ActionBarDrawerToggle drawerToggle;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);
        Toolbar toolbar = findViewById(R.id.toolbar);
        setSupportActionBar(toolbar);

        drawerLayout = findViewById(R.id.drawer_layout);
        drawerToggle = new ActionBarDrawerToggle(this, drawerLayout, toolbar, R.string.navigation_drawer_open, R.string.navigation_drawer_close);
        drawerLayout.addDrawerListener(drawerToggle);
        drawerToggle.syncState();

        NavigationView navigationView = findViewById(R.id.nav_view);
        navigationView.setNavigationItemSelectedListener(this);

        getSupportFragmentManager().addOnBackStackChangedListener(this);
        if (savedInstanceState == null) {
            getSupportFragmentManager().beginTransaction().add(R.id.fragment, new DevicesFragment(), "devices").commit();
        } else {
            onBackStackChanged();
        }
    }

    @Override
    public void onBackStackChanged() {
        getSupportActionBar().setDisplayHomeAsUpEnabled(getSupportFragmentManager().getBackStackEntryCount() > 0);
    }

    @Override
    public boolean onSupportNavigateUp() {
        onBackPressed();
        return true;
    }

    @Override
    public void onBackPressed() {
        if (drawerLayout.isDrawerOpen(GravityCompat.START)) {
            drawerLayout.closeDrawer(GravityCompat.START);
        } else {
            super.onBackPressed();
        }
    }

    @Override
    public boolean onNavigationItemSelected(MenuItem item) {
        int id = item.getItemId();

        Fragment fragment = null;
        String tag = "";

        if (id == R.id.nav_terminal) {
            SharedPreferences sharedPreferences = getSharedPreferences("prefs", Context.MODE_PRIVATE);
            String lastDevice = sharedPreferences.getString("last_device", null);
            if (lastDevice != null) {
                Bundle args = new Bundle();
                args.putString("device", lastDevice);
                fragment = new TerminalFragment();
                fragment.setArguments(args);
                tag = "terminal";
            }
        } else if (id == R.id.nav_devices) {
            fragment = new DevicesFragment();
            tag = "devices";
        } else if (id == R.id.nav_settings) {
            fragment = new SettingsFragment();
            tag = "settings";
        } else if (id == R.id.nav_pid_tuning) {
            fragment = new PidTuningFragment();
            tag = "pid_tuning";
        }

        if (fragment != null) {
            getSupportFragmentManager().beginTransaction().replace(R.id.fragment, fragment, tag).addToBackStack(null).commit();
        }

        drawerLayout.closeDrawer(GravityCompat.START);
        return true;
    }
}
